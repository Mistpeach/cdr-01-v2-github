// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware DeltaRobotController.cpp/.h by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Changes (Apache-2.0 4(b)): extracted a single-side 3-channel motion engine;
// servo output replaced by ServoHAL (LEDC); responses tagged and routed by the caller.

#pragma once
// ============================================================
// DeltaRobot —— 单侧 delta 机器人运动引擎（3 舵机）
// 移植自原版 DeltaRobotController 的全部运动逻辑：
//   IK/FK、关节/笛卡尔插值、轨迹（圆/利萨茹/玫瑰）、REPEAT 状态机、
//   Approach、T-Code Live、随机动作、工作空间/IK 校验。
// 每侧一个实例（左侧物理通道 0..2、右侧 3..5），状态完全独立。
// 全局功能（BLE/WiFi/LED/电池/T-Code 轴槽位持久化）留在 RobotController。
// ============================================================

#include "config.h"
#include "DeltaCommandParser.h"
#include "ServoHAL.h"
#include <functional>

class DeltaRobot {
public:
  // servo: 共享的 ServoHAL；physOffset: 本侧物理通道偏移（左=0，右=3）
  // tag: 响应标签（"L"/"R"）；responder: 响应输出回调；
  // vibratorSetter: 振动电机回调（percent, report）
  void init(ServoHAL* servo, int physOffset, const char* tag,
            std::function<void(const String&)> responder,
            std::function<void(int, bool)> vibratorSetter);

  // ---- 周期调度 ----
  void tick();          // 随机重复动作的切换检查（每循环调用）
  void executeStep();   // 10ms 步进（每节拍调用）

  // ---- 命令 ----
  // 运动类命令（POINT/LINE/JOINT/CIRCLE/…/REPEAT_*/VIBRATE/STOP）。
  // 返回 true 表示命令被识别并执行（含失败响应）。
  bool executeCommand(const ParsedCommand& parsed);
  void stopAll();                       // 急停（舵机保持最后位置，清 REPEAT/随机）
  void resetTo90(int motionMs);         // 按键回中
  void goHome(int motionMs);            // POINT 0 0 0
  void startRandomWeak();
  void startRandomStrong();
  void addDefaultRepeatCommand(const String& cmd);

  // ---- T-Code Live（本侧独立）----
  void applyTCodeAxes(const ParsedTCodeCommand& cmd);  // 用本侧轴槽位更新并启动 live 运动
  void tcodeTimeoutStop();               // 750ms 超时：停止 live 运动（不动振动）
  void tcodeStop();                      // DSTOP：清动作/随机状态
  bool isTCodeLive() const;

  // ---- T-Code 轴槽位配置（本侧 3 个槽位）----
  void resetAxisSlots();                 // 默认 L0=X、L1=Z、L2=Y，-100..100，不反相
  void setAxisSlot(int slot, char axisType, int channel, int coord,
                   bool invert, float minPercent, float maxPercent);
  void getAxisSlot(int slot, char& axisType, int& channel, int& coord,
                   bool& invert, float& minPercent, float& maxPercent) const;
  String axisLabel(int slot) const;
  const char* coordName(int coord) const;

private:
  struct CartesianPoint {
    float x;
    float y;
    float z;
  };

  struct RepeatLineCommand {
    CartesianPoint start;
    CartesianPoint end;
    float motionTime;
    float waitAfterStart;
    float waitAfterEnd;
  };

  struct RepeatTriangleCommand {
    CartesianPoint points[3];
    float motionTime;
    float waits[3];
  };

  enum class MotionMode {
    Idle,
    JointInterpolation,
    CartesianLine,
    Approach,
    Trajectory,
    TCodeLive,
    Wait,
  };

  enum class TrajectoryType {
    None,
    CircleXY,
    Circle3D,
    Lissajous,
    Rose,
  };

  enum class RepeatMode {
    None,
    Point,
    Line,
    Triangle,
    Circle,
  };

  // ---- 几何常量（config.h，与原版值一致）----
  const float baseRadius = DELTA_BASE_RADIUS;
  const float bicepLength = DELTA_BICEP_LENGTH;
  const float forearmLength = DELTA_FOREARM_LENGTH;
  const float effectorRadius = DELTA_EFFECTOR_RADIUS;
  const float homeX = DELTA_HOME_X;
  const float homeY = DELTA_HOME_Y;
  const float homeZ = DELTA_HOME_Z;
  const float workspaceRadius = DELTA_WORKSPACE_RADIUS;
  const float workspaceHeight = DELTA_WORKSPACE_HEIGHT;

  const int randomTimeMin = RANDOM_TIME_MIN_MS;
  const int randomTimeMax = RANDOM_TIME_MAX_MS;
  const float circleApproachPercent = CIRCLE_APPROACH_PERCENT;
  const float circleApproachMinTime = CIRCLE_APPROACH_MIN_MS;
  const float circleApproachMaxTime = CIRCLE_APPROACH_MAX_MS;
  const float approachDistanceEpsilon = 0.5;
  const float tcodeDefaultMotionTime = TCODE_DEFAULT_MOTION_MS;
  const float tcodeMinMotionTime = TCODE_MIN_MOTION_MS;
  const float tcodeMaxMotionTime = TCODE_MAX_MOTION_MS;

  // ---- REPEAT_RANDOM 预设（与原版一致）----
  static constexpr int NUM_COMMANDS_WEAK = 11;
  String command_list_weak[NUM_COMMANDS_WEAK] = {
    "REPEAT_LINE 0 0 -50 0 0 50 300 0 500",
    "REPEAT_LINE 75 0 25 -75 0 25 800",
    "REPEAT_LINE 0 75 25 0 -75 25 800",
    "REPEAT_LINE 40 0 -50 -40 0 -50 400",
    "REPEAT_LINE 0 40 -50 0 -40 -50 400",
    "REPEAT_CIRCLE 0 0 -50 25 1000",
    "REPEAT_CIRCLE 0 0 -25 75 2000",
    "REPEAT_CIRCLE 0 0 -25 -75 2000",
    "REPEAT_TRIANGLE 0 0 90 0 100 -50 0 -100 -50 400 0 200 0",
    "REPEAT_LISSAJOUS 0 0 -20 35 35 15 3 2 1 90 180 4000",
    "REPEAT_ROSE 0 0 -25 50 4 6000"
  };

  static constexpr int NUM_COMMANDS_STRONG = 10;
  String command_list_strong[NUM_COMMANDS_STRONG] = {
    "REPEAT_LINE 0 0 -50 0 0 50 200 0 200",
    "REPEAT_LINE 0 0 -50 0 0 50 200 0 100",
    "REPEAT_LINE 75 0 25 -75 0 25 300",
    "REPEAT_LINE 0 75 25 0 -75 25 300",
    "REPEAT_CIRCLE 0 0 -25 25 300",
    "REPEAT_CIRCLE 0 0 -25 -25 300",
    "REPEAT_TRIANGLE 0 0 90 0 100 -50 0 -100 -50 150 0 0 0",
    "REPEAT_LISSAJOUS 0 0 -20 35 35 15 3 2 1 90 180 1500",
    "REPEAT_ROSE 0 0 -25 50 4 2000",
    "REPEAT_LINE 25 0 -50 -25 0 -50 60"
  };

  // ---- HAL 引用 ----
  ServoHAL* _servo = nullptr;
  int _physOffset = 0;
  const char* _tag = "?";
  std::function<void(const String&)> _responder;
  std::function<void(int, bool)> _vibratorSetter;

  // ---- 状态 ----
  float currentAngles[3] = {90, 90, 90};
  float targetAngles[3] = {90, 90, 90};
  float stepAngles[3] = {0, 0, 0};
  CartesianPoint currentPosition = {0, 0, -33};
  CartesianPoint pendingPosition = {0, 0, -33};
  CartesianPoint lineStartPosition = {0, 0, -33};
  CartesianPoint lineEndPosition = {0, 0, -33};
  bool currentPositionValid = true;
  bool pendingPositionValid = false;
  float tcodeLinearValues[3] = {0.5, 0.5, 0.5};
  CartesianPoint tcodeLiveStartPosition = {0, 0, -33};
  CartesianPoint tcodeLiveTargetPosition = {0, 0, -33};
  int tcodeLiveTotalSteps = 1;
  int tcodeLiveCurrentStep = 0;

  char tcodeAxisType[3] = {'L', 'L', 'L'};
  int tcodeAxisChannel[3] = {0, 1, 2};
  int tcodeAxisCoord[3] = {0, 2, 1};
  bool tcodeAxisInvert[3] = {false, false, false};
  float tcodeAxisMinPercent[3] = {-100.0, -100.0, -100.0};
  float tcodeAxisMaxPercent[3] = {100.0, 100.0, 100.0};

  int totalSteps = 0;
  int currentStep = 0;

  String repeatCommand = "";
  MotionMode motionMode = MotionMode::Idle;
  RepeatMode repeatMode = RepeatMode::None;
  MotionMode approachNextMotionMode = MotionMode::Idle;

  TrajectoryType trajectoryType = TrajectoryType::None;
  CartesianPoint trajectoryCenter = {0, 0, -33};
  CartesianPoint trajectoryU = {1, 0, 0};
  CartesianPoint trajectoryV = {0, 1, 0};
  CartesianPoint lissajousAmplitude = {0, 0, 0};
  CartesianPoint lissajousFrequency = {1, 1, 1};
  float trajectoryRadius = 0;
  float lissajousPhaseY = 0;
  float lissajousPhaseZ = 0;
  float rosePetals = 4;
  int trajectorySteps = 0;

  static constexpr int MAX_REPEAT_COMMANDS = 30;
  String commandList[MAX_REPEAT_COMMANDS];
  int commandCount = 0;
  bool isRandomRepeat = false;
  unsigned long randomSwitchTime = 0;
  unsigned long waitUntilTime = 0;
  bool repeatWaitJustCompleted = false;
  bool repeatLineToggle = false;
  int repeatTriangleTargetIndex = 0;

  // ---- 响应 ----
  void sendOk(const String& message);
  void sendError(const String& code, const String& message);

  void dispatchCommand(const ParsedCommand& command);
  void handlePointCommand(const ParsedCommand& command);
  void handleLineCommand(const ParsedCommand& command);
  void handleJointCommand(const ParsedCommand& command);
  void handleCircleCommand(const ParsedCommand& command, bool repeating);
  void handleCircle3DCommand(const ParsedCommand& command, bool repeating);
  void handleLissajousCommand(const ParsedCommand& command, bool repeating);
  void handleRoseCommand(const ParsedCommand& command, bool repeating);
  void handleRepeatPointCommand(const ParsedCommand& command);
  void handleRepeatLineCommand(const ParsedCommand& command);
  void handleRepeatTriangleCommand(const ParsedCommand& command);
  void handleRepeatRandomCommand();
  void handleVibrateCommand(const ParsedCommand& command);
  void handleStopCommand();

  void executeJointInterpolationStep();
  void executeCartesianLineStep();
  void executeTrajectoryStep();
  void executeTCodeLiveStep();
  void handleCompletedMotion();
  bool isWaiting();
  void clearActiveMotion();

  CartesianPoint parsedPercentPoint(const ParsedCommand& command, int startIndex);
  RepeatLineCommand parsedRepeatLineCommand(const ParsedCommand& command);
  RepeatTriangleCommand parsedRepeatTriangleCommand(const ParsedCommand& command);
  bool startRepeatWaitIfNeeded(const RepeatLineCommand& repeatLine);
  bool startRepeatTriangleWaitIfNeeded(const RepeatTriangleCommand& triangle);
  bool validateRepeatTriangleCommand(const RepeatTriangleCommand& triangle);
  bool shouldApproachRepeatStart(const CartesianPoint& repeatStart) const;
  bool startRepeatPointInitialMotion(const RepeatLineCommand& repeatLine);
  bool startRepeatLineInitialMotion(const RepeatLineCommand& repeatLine);
  bool startRepeatTriangleInitialMotion(const RepeatTriangleCommand& triangle);
  bool moveToCartesianTarget(const CartesianPoint& target, float motionTime);
  bool startCartesianLineMotion(const CartesianPoint& start, const CartesianPoint& target, float motionTime, MotionMode mode);
  bool startCircleMotion(const CartesianPoint& center, float radius, float motionTime, bool repeating);
  bool startCircle3DMotion(const CartesianPoint& center, const CartesianPoint& normal, float radius, float motionTime, bool repeating);
  bool startLissajousMotion(const CartesianPoint& center, const CartesianPoint& amplitude, const CartesianPoint& frequency, float phaseY, float phaseZ, float motionTime, bool repeating);
  bool startRoseMotion(const CartesianPoint& center, float radius, float petals, float motionTime, bool repeating);
  bool startTrajectoryMotion(TrajectoryType type, float motionTime, bool repeating);
  bool validateTrajectorySamples();
  bool shouldApproachCircleStart(const CartesianPoint& circleStart) const;
  CartesianPoint trajectoryStartPoint();
  bool sampleTrajectory(float t, CartesianPoint& point);
  bool setupPlaneBasis(const CartesianPoint& normal);
  CartesianPoint makePoint(float x, float y, float z) const;
  CartesianPoint addPoints(const CartesianPoint& a, const CartesianPoint& b) const;
  CartesianPoint scalePoint(const CartesianPoint& point, float scale) const;
  CartesianPoint crossProduct(const CartesianPoint& a, const CartesianPoint& b) const;
  float dotProduct(const CartesianPoint& a, const CartesianPoint& b) const;
  float vectorLength(const CartesianPoint& point) const;
  bool normalizePoint(CartesianPoint& point) const;
  float circleApproachTime(float motionTime) const;
  float distanceBetween(const CartesianPoint& a, const CartesianPoint& b) const;
  void markPendingPosition(const CartesianPoint& target);
  void clearPendingPosition();
  void prepareInterpolation(float target[], float speed);

  void debugPrintAngles();
  void stopServos();
  void setServoAngle(int channel, float angle);
  int angleToPulse(float angle);
  int motionTimeToSteps(float motionTime) const;
  CartesianPoint percentXYZToXYZ(float percent_x, float percent_y, float percent_z);
  float percentRadiusToRadius(float percent_r);
  int inverseKinematics(float x0, float y0, float z0, float angles[]);
  bool forwardKinematics(float a1, float a2, float a3, CartesianPoint& point) const;
  bool markPendingPositionFromJointAngles(float a1, float a2, float a3);
  int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
  bool isWithinWorkspace(float x, float y, float z);

  void addRepeatCommand(const String& cmd);
  void selectAndRunRandomCommand();
  void setRepeatCommands(const String commands[], int count);
  void processInternalCommand(const String& command);

  float tcodeMotionTimeForAxis(int axis, const TCodeAxisCommand& command) const;
  float tcodeMagnitudeToPercent(int axis, float magnitude) const;
  CartesianPoint tcodeTargetFromState();
  void startTCodeLiveMotion(const CartesianPoint& target, float motionTime);
  void stopTCodeLiveMotion(bool stopVibration);
  const TCodeAxisCommand* findTCodeAxisCommand(const ParsedTCodeCommand& command, char axisType, int channel) const;
};
