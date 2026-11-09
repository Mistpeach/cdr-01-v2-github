// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware DeltaRobotController.cpp/.h by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Changes (Apache-2.0 4(b)): extracted a single-side 3-channel motion engine;
// servo output replaced by ServoHAL (LEDC); responses tagged and routed by the caller.

#include "DeltaRobot.h"

#include <ctype.h>
#include <string.h>

// ============================================================
// 单侧 delta 运动引擎（移植自原版 DeltaRobotController.cpp，Apache-2.0）
// 差异：setServoAngle 走 ServoHAL 物理通道（physOffset + channel）；
//       响应带 [L]/[R] 标签经回调输出；振动经共享回调。
// ============================================================

void DeltaRobot::init(ServoHAL* servo, int physOffset, const char* tag,
                      std::function<void(const String&)> responder,
                      std::function<void(int, bool)> vibratorSetter) {
  _servo = servo;
  _physOffset = physOffset;
  _tag = tag;
  _responder = responder;
  _vibratorSetter = vibratorSetter;
}

void DeltaRobot::sendOk(const String& message) {
  String response = "[";
  response += _tag;
  response += "] OK ";
  response += message;
  if (_responder) _responder(response);
}

void DeltaRobot::sendError(const String& code, const String& message) {
  String response = "[";
  response += _tag;
  response += "] ERR ";
  response += code;
  response += " ";
  response += message;
  if (_responder) _responder(response);
}

void DeltaRobot::tick() {
  if (isRandomRepeat && millis() > randomSwitchTime) {
    randomSwitchTime = millis() + random(randomTimeMin, randomTimeMax);
    selectAndRunRandomCommand();
  }
}

void DeltaRobot::executeStep() {
  if (isWaiting()) return;

  if (motionMode == MotionMode::TCodeLive) {
    executeTCodeLiveStep();
    return;
  }

  if (currentStep < totalSteps) {
    if (motionMode == MotionMode::Trajectory) {
      executeTrajectoryStep();
    } else if (motionMode == MotionMode::CartesianLine || motionMode == MotionMode::Approach) {
      executeCartesianLineStep();
    } else if (motionMode == MotionMode::JointInterpolation) {
      executeJointInterpolationStep();
    }

    currentStep++;
    if (currentStep >= totalSteps) handleCompletedMotion();
  } else {
    handleCompletedMotion();
  }
}

bool DeltaRobot::executeCommand(const ParsedCommand& parsed) {
  switch (parsed.type) {
    case CommandType::Point:
      handlePointCommand(parsed);
      return true;
    case CommandType::Line:
      handleLineCommand(parsed);
      return true;
    case CommandType::Joint:
      handleJointCommand(parsed);
      return true;
    case CommandType::Circle:
      handleCircleCommand(parsed, false);
      return true;
    case CommandType::Circle3D:
      handleCircle3DCommand(parsed, false);
      return true;
    case CommandType::Lissajous:
      handleLissajousCommand(parsed, false);
      return true;
    case CommandType::Rose:
      handleRoseCommand(parsed, false);
      return true;
    case CommandType::RepeatPoint:
      handleRepeatPointCommand(parsed);
      return true;
    case CommandType::RepeatLine:
      handleRepeatLineCommand(parsed);
      return true;
    case CommandType::RepeatTriangle:
      handleRepeatTriangleCommand(parsed);
      return true;
    case CommandType::RepeatCircle:
      handleCircleCommand(parsed, true);
      return true;
    case CommandType::RepeatCircle3D:
      handleCircle3DCommand(parsed, true);
      return true;
    case CommandType::RepeatLissajous:
      handleLissajousCommand(parsed, true);
      return true;
    case CommandType::RepeatRose:
      handleRoseCommand(parsed, true);
      return true;
    case CommandType::RepeatRandom:
      handleRepeatRandomCommand();
      return true;
    case CommandType::Vibrate:
      handleVibrateCommand(parsed);
      return true;
    case CommandType::Stop:
      handleStopCommand();
      return true;
    default:
      return false;
  }
}

void DeltaRobot::stopAll() {
  clearActiveMotion();
  isRandomRepeat = false;
}

void DeltaRobot::resetTo90(int motionMs) {
  ParsedCommand parsed;
  String cmd = "JOINT 90 90 90 " + String(motionMs);
  if (parseCommand(cmd.c_str(), parsed)) {
    clearActiveMotion();
    handleJointCommand(parsed);
  }
}

void DeltaRobot::goHome(int motionMs) {
  ParsedCommand parsed;
  String cmd = "POINT 0 0 0 " + String(motionMs);
  if (parseCommand(cmd.c_str(), parsed)) {
    clearActiveMotion();
    handlePointCommand(parsed);
  }
}

void DeltaRobot::startRandomWeak() {
  setRepeatCommands(command_list_weak, NUM_COMMANDS_WEAK);
  handleRepeatRandomCommand();
}

void DeltaRobot::startRandomStrong() {
  setRepeatCommands(command_list_strong, NUM_COMMANDS_STRONG);
  handleRepeatRandomCommand();
}

void DeltaRobot::addDefaultRepeatCommand(const String& cmd) {
  addRepeatCommand(cmd);
}

void DeltaRobot::handlePointCommand(const ParsedCommand& command) {
  CartesianPoint target = parsedPercentPoint(command, 0);
  if (moveToCartesianTarget(target, command.values[3])) sendOk("POINT started");
}

void DeltaRobot::handleLineCommand(const ParsedCommand& command) {
  CartesianPoint target = parsedPercentPoint(command, 0);
  if (!currentPositionValid) {
    sendError("POSITION_UNKNOWN", "Current Cartesian position is unknown. Use POINT first.");
    return;
  }

  if (startCartesianLineMotion(currentPosition, target, command.values[3], MotionMode::CartesianLine)) {
    sendOk("LINE started");
  }
}

void DeltaRobot::handleJointCommand(const ParsedCommand& command) {
  targetAngles[0] = command.values[0];
  targetAngles[1] = command.values[1];
  targetAngles[2] = command.values[2];

  if (!markPendingPositionFromJointAngles(targetAngles[0], targetAngles[1], targetAngles[2])) {
    clearPendingPosition();
  }
  debugPrintAngles();
  prepareInterpolation(targetAngles, command.values[3]);
  sendOk("JOINT started");
}

void DeltaRobot::handleCircleCommand(const ParsedCommand& command, bool repeating) {
  CartesianPoint center = parsedPercentPoint(command, 0);
  float radius = percentRadiusToRadius(command.values[3]);

  if (startCircleMotion(center, radius, command.values[4], repeating)) {
    sendOk(repeating ? "REPEAT_CIRCLE started" : "CIRCLE started");
  }
}

void DeltaRobot::handleCircle3DCommand(const ParsedCommand& command, bool repeating) {
  CartesianPoint center = parsedPercentPoint(command, 0);
  CartesianPoint normal = makePoint(command.values[3], command.values[4], command.values[5]);
  float radius = percentRadiusToRadius(command.values[6]);

  if (startCircle3DMotion(center, normal, radius, command.values[7], repeating)) {
    sendOk(repeating ? "REPEAT_CIRCLE3D started" : "CIRCLE3D started");
  }
}

void DeltaRobot::handleLissajousCommand(const ParsedCommand& command, bool repeating) {
  CartesianPoint center = parsedPercentPoint(command, 0);
  CartesianPoint amplitude = makePoint(
    percentRadiusToRadius(command.values[3]),
    percentRadiusToRadius(command.values[4]),
    command.values[5] / 100.0 * workspaceHeight
  );
  CartesianPoint frequency = makePoint(command.values[6], command.values[7], command.values[8]);

  if (startLissajousMotion(center, amplitude, frequency, command.values[9], command.values[10], command.values[11], repeating)) {
    sendOk(repeating ? "REPEAT_LISSAJOUS started" : "LISSAJOUS started");
  }
}

void DeltaRobot::handleRoseCommand(const ParsedCommand& command, bool repeating) {
  CartesianPoint center = parsedPercentPoint(command, 0);
  float radius = percentRadiusToRadius(command.values[3]);

  if (startRoseMotion(center, radius, command.values[4], command.values[5], repeating)) {
    sendOk(repeating ? "REPEAT_ROSE started" : "ROSE started");
  }
}

void DeltaRobot::handleRepeatPointCommand(const ParsedCommand& command) {
  RepeatLineCommand repeatLine = parsedRepeatLineCommand(command);

  repeatCommand = "";
  for (int i = 0; i < command.valueCount; i++) {
    if (i > 0) repeatCommand += " ";
    repeatCommand += String(command.values[i], 6);
  }

  repeatLineToggle = true;
  if (startRepeatPointInitialMotion(repeatLine)) {
    sendOk("REPEAT_POINT started");
  }
}

void DeltaRobot::handleRepeatLineCommand(const ParsedCommand& command) {
  RepeatLineCommand repeatLine = parsedRepeatLineCommand(command);

  if (!currentPositionValid) {
    sendError("POSITION_UNKNOWN", "Current Cartesian position is unknown. Use POINT first.");
    return;
  }

  repeatCommand = "";
  for (int i = 0; i < command.valueCount; i++) {
    if (i > 0) repeatCommand += " ";
    repeatCommand += String(command.values[i], 6);
  }

  repeatLineToggle = true;
  if (startRepeatLineInitialMotion(repeatLine)) {
    sendOk("REPEAT_LINE started");
  }
}

void DeltaRobot::handleRepeatTriangleCommand(const ParsedCommand& command) {
  RepeatTriangleCommand triangle = parsedRepeatTriangleCommand(command);

  if (!currentPositionValid) {
    sendError("POSITION_UNKNOWN", "Current Cartesian position is unknown. Use POINT first.");
    return;
  }

  if (!validateRepeatTriangleCommand(triangle)) return;

  repeatCommand = "";
  for (int i = 0; i < command.valueCount; i++) {
    if (i > 0) repeatCommand += " ";
    repeatCommand += String(command.values[i], 6);
  }

  repeatTriangleTargetIndex = 0;
  if (startRepeatTriangleInitialMotion(triangle)) {
    sendOk("REPEAT_TRIANGLE started");
  }
}

void DeltaRobot::handleRepeatRandomCommand() {
  isRandomRepeat = true;
  randomSwitchTime = millis() + random(randomTimeMin, randomTimeMax);
  selectAndRunRandomCommand();
  sendOk("REPEAT_RANDOM started");
}

void DeltaRobot::handleVibrateCommand(const ParsedCommand& command) {
  if (_vibratorSetter) _vibratorSetter(command.intValue, true);
  sendOk("VIBRATE set");
}

void DeltaRobot::handleStopCommand() {
  clearActiveMotion();
  isRandomRepeat = false;
}

void DeltaRobot::executeJointInterpolationStep() {
  for (int i = 0; i < 3; i++) {
    currentAngles[i] += stepAngles[i];
    setServoAngle(i, currentAngles[i]);
  }
}

void DeltaRobot::executeCartesianLineStep() {
  float t = (float)(currentStep + 1) / totalSteps;
  CartesianPoint target;
  target.x = lineStartPosition.x + (lineEndPosition.x - lineStartPosition.x) * t;
  target.y = lineStartPosition.y + (lineEndPosition.y - lineStartPosition.y) * t;
  target.z = lineStartPosition.z + (lineEndPosition.z - lineStartPosition.z) * t;

  if (!isWithinWorkspace(target.x, target.y, target.z)) {
    sendError("WORKSPACE", "Cartesian line out of workspace. Motion halted.");
    stopServos();
    clearActiveMotion();
    return;
  }

  if (inverseKinematics(target.x, target.y, target.z, targetAngles) != 0) {
    sendError("IK", "Inverse kinematics failed. Motion halted.");
    stopServos();
    clearActiveMotion();
    return;
  }

  for (int i = 0; i < 3; i++) setServoAngle(i, targetAngles[i]);
  currentPosition = target;
  currentPositionValid = true;
}

void DeltaRobot::executeTCodeLiveStep() {
  if (tcodeLiveCurrentStep >= tcodeLiveTotalSteps) return;

  float t = (float)(tcodeLiveCurrentStep + 1) / tcodeLiveTotalSteps;
  CartesianPoint target;
  target.x = tcodeLiveStartPosition.x + (tcodeLiveTargetPosition.x - tcodeLiveStartPosition.x) * t;
  target.y = tcodeLiveStartPosition.y + (tcodeLiveTargetPosition.y - tcodeLiveStartPosition.y) * t;
  target.z = tcodeLiveStartPosition.z + (tcodeLiveTargetPosition.z - tcodeLiveStartPosition.z) * t;

  if (!isWithinWorkspace(target.x, target.y, target.z)) {
    stopTCodeLiveMotion(false);
    return;
  }

  if (inverseKinematics(target.x, target.y, target.z, targetAngles) != 0) {
    stopTCodeLiveMotion(false);
    return;
  }

  for (int i = 0; i < 3; i++) setServoAngle(i, targetAngles[i]);
  currentPosition = target;
  currentPositionValid = true;
  tcodeLiveCurrentStep++;
}

void DeltaRobot::executeTrajectoryStep() {
  CartesianPoint target;
  float t = (float)currentStep / trajectorySteps;
  if (!sampleTrajectory(t, target)) {
    sendError("TRAJECTORY", "Trajectory sample failed. Motion halted.");
    stopServos();
    clearActiveMotion();
    return;
  }

  if (!isWithinWorkspace(target.x, target.y, target.z)) {
    sendError("WORKSPACE", "Trajectory out of workspace. Motion halted.");
    stopServos();
    clearActiveMotion();
    return;
  }

  if (inverseKinematics(target.x, target.y, target.z, targetAngles) == 0) {
    for (int i = 0; i < 3; i++) setServoAngle(i, targetAngles[i]);
    currentPosition = target;
    currentPositionValid = true;
  } else {
    sendError("IK", "Inverse kinematics failed. Motion halted.");
    stopServos();
    clearActiveMotion();
  }
}

void DeltaRobot::handleCompletedMotion() {
  if (motionMode == MotionMode::JointInterpolation) {
    if (pendingPositionValid) {
      currentPosition = pendingPosition;
      currentPositionValid = true;
    } else {
      currentPositionValid = false;
    }
    clearPendingPosition();
  } else if (motionMode == MotionMode::CartesianLine || motionMode == MotionMode::Approach) {
    currentPosition = lineEndPosition;
    currentPositionValid = true;
  }

  if (motionMode == MotionMode::Approach) {
    motionMode = approachNextMotionMode;
    totalSteps = trajectorySteps;
    currentStep = 0;
    return;
  }

  if (repeatMode == RepeatMode::Point || repeatMode == RepeatMode::Line) {
    ParsedCommand parsed;
    String command = "";
    if (repeatMode == RepeatMode::Point) {
      command += "REPEAT_POINT ";
    } else {
      command += "REPEAT_LINE ";
    }
    command += repeatCommand;
    if (!parseCommand(command.c_str(), parsed)) {
      sendError("INVALID_REPEAT", "Invalid repeat command. Motion halted.");
      clearActiveMotion();
      return;
    }

    RepeatLineCommand repeatLine = parsedRepeatLineCommand(parsed);
    if (!repeatWaitJustCompleted && startRepeatWaitIfNeeded(repeatLine)) return;
    repeatWaitJustCompleted = false;

    repeatLineToggle = !repeatLineToggle;
    const CartesianPoint& target = repeatLineToggle ? repeatLine.start : repeatLine.end;
    bool started = false;
    if (repeatMode == RepeatMode::Point) {
      started = moveToCartesianTarget(target, repeatLine.motionTime);
    } else if (currentPositionValid) {
      started = startCartesianLineMotion(currentPosition, target, repeatLine.motionTime, MotionMode::CartesianLine);
    } else {
      sendError("POSITION_UNKNOWN", "Current Cartesian position is unknown. Repeat halted.");
    }
    if (!started) clearActiveMotion();
  } else if (repeatMode == RepeatMode::Triangle) {
    ParsedCommand parsed;
    String command = "REPEAT_TRIANGLE ";
    command += repeatCommand;
    if (!parseCommand(command.c_str(), parsed)) {
      sendError("INVALID_REPEAT", "Invalid repeat command. Motion halted.");
      clearActiveMotion();
      return;
    }

    RepeatTriangleCommand triangle = parsedRepeatTriangleCommand(parsed);
    if (!repeatWaitJustCompleted && startRepeatTriangleWaitIfNeeded(triangle)) return;
    repeatWaitJustCompleted = false;

    repeatTriangleTargetIndex = (repeatTriangleTargetIndex + 1) % 3;
    bool started = false;
    if (currentPositionValid) {
      started = startCartesianLineMotion(currentPosition, triangle.points[repeatTriangleTargetIndex], triangle.motionTime, MotionMode::CartesianLine);
    } else {
      sendError("POSITION_UNKNOWN", "Current Cartesian position is unknown. Repeat halted.");
    }
    if (!started) clearActiveMotion();
  } else if (repeatMode == RepeatMode::Circle) {
    currentStep = 0;
  } else if (motionMode != MotionMode::Idle) {
    motionMode = MotionMode::Idle;
  }
}

void DeltaRobot::clearActiveMotion() {
  repeatMode = RepeatMode::None;
  motionMode = MotionMode::Idle;
  approachNextMotionMode = MotionMode::Idle;
  trajectorySteps = 0;
  trajectoryType = TrajectoryType::None;
  currentStep = totalSteps;
  waitUntilTime = 0;
  repeatWaitJustCompleted = false;
  repeatTriangleTargetIndex = 0;
  clearPendingPosition();
}

bool DeltaRobot::isWaiting() {
  if (motionMode != MotionMode::Wait) return false;
  if (millis() < waitUntilTime) return true;

  motionMode = MotionMode::Idle;
  waitUntilTime = 0;
  repeatWaitJustCompleted = true;
  return false;
}

DeltaRobot::CartesianPoint DeltaRobot::parsedPercentPoint(const ParsedCommand& command, int startIndex) {
  return percentXYZToXYZ(command.values[startIndex], command.values[startIndex + 1], command.values[startIndex + 2]);
}

DeltaRobot::RepeatLineCommand DeltaRobot::parsedRepeatLineCommand(const ParsedCommand& command) {
  RepeatLineCommand repeatLine;
  repeatLine.start = parsedPercentPoint(command, 0);
  repeatLine.end = parsedPercentPoint(command, 3);
  repeatLine.motionTime = command.values[6];
  repeatLine.waitAfterStart = command.valueCount >= 9 ? command.values[7] : 0.0;
  repeatLine.waitAfterEnd = command.valueCount >= 9 ? command.values[8] : 0.0;
  return repeatLine;
}

DeltaRobot::RepeatTriangleCommand DeltaRobot::parsedRepeatTriangleCommand(const ParsedCommand& command) {
  RepeatTriangleCommand triangle;
  triangle.points[0] = parsedPercentPoint(command, 0);
  triangle.points[1] = parsedPercentPoint(command, 3);
  triangle.points[2] = parsedPercentPoint(command, 6);
  triangle.motionTime = command.values[9];
  for (int i = 0; i < 3; i++) {
    triangle.waits[i] = command.valueCount >= 13 ? command.values[10 + i] : 0.0;
  }
  return triangle;
}

bool DeltaRobot::startRepeatWaitIfNeeded(const RepeatLineCommand& repeatLine) {
  float waitTime = repeatLineToggle ? repeatLine.waitAfterStart : repeatLine.waitAfterEnd;
  if (waitTime <= 0) return false;

  motionMode = MotionMode::Wait;
  waitUntilTime = millis() + (unsigned long)waitTime;
  repeatWaitJustCompleted = false;
  currentStep = totalSteps;
  return true;
}

bool DeltaRobot::startRepeatTriangleWaitIfNeeded(const RepeatTriangleCommand& triangle) {
  int index = repeatTriangleTargetIndex;
  if (index < 0 || index >= 3) index = 0;

  float waitTime = triangle.waits[index];
  if (waitTime <= 0) return false;

  motionMode = MotionMode::Wait;
  waitUntilTime = millis() + (unsigned long)waitTime;
  repeatWaitJustCompleted = false;
  currentStep = totalSteps;
  return true;
}

bool DeltaRobot::validateRepeatTriangleCommand(const RepeatTriangleCommand& triangle) {
  for (int i = 0; i < 3; i++) {
    const CartesianPoint& point = triangle.points[i];
    if (!isWithinWorkspace(point.x, point.y, point.z)) {
      sendError("WORKSPACE", "Triangle point out of workspace. Command ignored.");
      return false;
    }
    if (inverseKinematics(point.x, point.y, point.z, targetAngles) != 0) {
      sendError("IK", "Triangle inverse kinematics failed. Command ignored.");
      return false;
    }
  }
  return true;
}

bool DeltaRobot::shouldApproachRepeatStart(const CartesianPoint& repeatStart) const {
  return currentPositionValid && distanceBetween(currentPosition, repeatStart) > approachDistanceEpsilon;
}

bool DeltaRobot::startRepeatPointInitialMotion(const RepeatLineCommand& repeatLine) {
  repeatLineToggle = true;

  if (currentPositionValid) {
    if (!shouldApproachRepeatStart(repeatLine.start)) {
      currentPosition = repeatLine.start;
      repeatMode = RepeatMode::Point;
      handleCompletedMotion();
      return repeatMode == RepeatMode::Point;
    }

    if (startCartesianLineMotion(currentPosition, repeatLine.start, circleApproachTime(repeatLine.motionTime), MotionMode::CartesianLine)) {
      repeatMode = RepeatMode::Point;
      return true;
    }
    return false;
  }

  if (moveToCartesianTarget(repeatLine.start, repeatLine.motionTime)) {
    repeatMode = RepeatMode::Point;
    return true;
  }
  return false;
}

bool DeltaRobot::startRepeatLineInitialMotion(const RepeatLineCommand& repeatLine) {
  repeatLineToggle = true;

  if (!shouldApproachRepeatStart(repeatLine.start)) {
    currentPosition = repeatLine.start;
    currentPositionValid = true;
    repeatMode = RepeatMode::Line;
    handleCompletedMotion();
    return repeatMode == RepeatMode::Line;
  }

  if (startCartesianLineMotion(currentPosition, repeatLine.start, circleApproachTime(repeatLine.motionTime), MotionMode::CartesianLine)) {
    repeatMode = RepeatMode::Line;
    return true;
  }
  return false;
}

bool DeltaRobot::startRepeatTriangleInitialMotion(const RepeatTriangleCommand& triangle) {
  repeatTriangleTargetIndex = 0;

  if (!shouldApproachRepeatStart(triangle.points[0])) {
    currentPosition = triangle.points[0];
    currentPositionValid = true;
    repeatMode = RepeatMode::Triangle;
    handleCompletedMotion();
    return repeatMode == RepeatMode::Triangle;
  }

  if (startCartesianLineMotion(currentPosition, triangle.points[0], circleApproachTime(triangle.motionTime), MotionMode::CartesianLine)) {
    repeatMode = RepeatMode::Triangle;
    return true;
  }
  return false;
}

bool DeltaRobot::moveToCartesianTarget(const CartesianPoint& target, float motionTime) {
  if (!isWithinWorkspace(target.x, target.y, target.z)) {
    sendError("WORKSPACE", "Target out of workspace. Command ignored.");
    return false;
  }

  if (inverseKinematics(target.x, target.y, target.z, targetAngles) != 0) {
    sendError("IK", "Inverse kinematics failed. Command ignored.");
    stopServos();
    return false;
  }

  markPendingPosition(target);
  debugPrintAngles();
  prepareInterpolation(targetAngles, motionTime);
  return true;
}

bool DeltaRobot::startCartesianLineMotion(const CartesianPoint& start, const CartesianPoint& target, float motionTime, MotionMode mode) {
  if (!isWithinWorkspace(start.x, start.y, start.z) || !isWithinWorkspace(target.x, target.y, target.z)) {
    sendError("WORKSPACE", "Cartesian line target out of workspace. Command ignored.");
    return false;
  }

  if (inverseKinematics(target.x, target.y, target.z, targetAngles) != 0) {
    sendError("IK", "Inverse kinematics failed. Command ignored.");
    stopServos();
    return false;
  }

  lineStartPosition = start;
  lineEndPosition = target;
  motionMode = mode;
  totalSteps = motionTimeToSteps(motionTime);
  currentStep = 0;
  clearPendingPosition();
  return true;
}

bool DeltaRobot::startCircleMotion(const CartesianPoint& center, float radius, float motionTime, bool repeating) {
  trajectoryCenter = center;
  trajectoryU = makePoint(1, 0, 0);
  trajectoryV = makePoint(0, 1, 0);
  trajectoryRadius = radius;
  return startTrajectoryMotion(TrajectoryType::CircleXY, motionTime, repeating);
}

bool DeltaRobot::startCircle3DMotion(const CartesianPoint& center, const CartesianPoint& normal, float radius, float motionTime, bool repeating) {
  trajectoryCenter = center;
  trajectoryRadius = radius;
  if (!setupPlaneBasis(normal)) {
    sendError("INVALID_ARGUMENT", "CIRCLE3D normal vector must be non-zero.");
    return false;
  }

  return startTrajectoryMotion(TrajectoryType::Circle3D, motionTime, repeating);
}

bool DeltaRobot::startLissajousMotion(const CartesianPoint& center, const CartesianPoint& amplitude, const CartesianPoint& frequency, float phaseY, float phaseZ, float motionTime, bool repeating) {
  trajectoryCenter = center;
  lissajousAmplitude = amplitude;
  lissajousFrequency = frequency;
  lissajousPhaseY = phaseY * PI / 180.0;
  lissajousPhaseZ = phaseZ * PI / 180.0;
  return startTrajectoryMotion(TrajectoryType::Lissajous, motionTime, repeating);
}

bool DeltaRobot::startRoseMotion(const CartesianPoint& center, float radius, float petals, float motionTime, bool repeating) {
  trajectoryCenter = center;
  trajectoryU = makePoint(1, 0, 0);
  trajectoryV = makePoint(0, 1, 0);
  trajectoryRadius = radius;
  rosePetals = petals;
  return startTrajectoryMotion(TrajectoryType::Rose, motionTime, repeating);
}

bool DeltaRobot::startTrajectoryMotion(TrajectoryType type, float motionTime, bool repeating) {
  trajectoryType = type;
  repeatMode = repeating ? RepeatMode::Circle : RepeatMode::None;
  trajectorySteps = motionTimeToSteps(motionTime);
  totalSteps = trajectorySteps;
  clearPendingPosition();

  if (!validateTrajectorySamples()) return false;

  CartesianPoint startPoint = trajectoryStartPoint();
  if (shouldApproachCircleStart(startPoint)) {
    approachNextMotionMode = MotionMode::Trajectory;
    if (startCartesianLineMotion(currentPosition, startPoint, circleApproachTime(motionTime), MotionMode::Approach)) return true;
    clearActiveMotion();
    return false;
  }

  motionMode = MotionMode::Trajectory;
  currentStep = 0;
  return true;
}

bool DeltaRobot::validateTrajectorySamples() {
  static const float samples[] = {0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875};
  CartesianPoint point;
  for (int i = 0; i < 8; i++) {
    if (!sampleTrajectory(samples[i], point)) {
      sendError("TRAJECTORY", "Trajectory sample failed. Command ignored.");
      return false;
    }
    if (!isWithinWorkspace(point.x, point.y, point.z)) {
      sendError("WORKSPACE", "Trajectory out of workspace. Command ignored.");
      return false;
    }
    if (inverseKinematics(point.x, point.y, point.z, targetAngles) != 0) {
      sendError("IK", "Trajectory inverse kinematics failed. Command ignored.");
      return false;
    }
  }

  return true;
}

bool DeltaRobot::shouldApproachCircleStart(const CartesianPoint& circleStart) const {
  return currentPositionValid && distanceBetween(currentPosition, circleStart) > approachDistanceEpsilon;
}

DeltaRobot::CartesianPoint DeltaRobot::trajectoryStartPoint() {
  CartesianPoint point;
  if (!sampleTrajectory(0.0, point)) return trajectoryCenter;
  return point;
}

bool DeltaRobot::sampleTrajectory(float t, CartesianPoint& point) {
  float angle = 2 * PI * t;

  if ((trajectoryType == TrajectoryType::CircleXY || trajectoryType == TrajectoryType::Circle3D) && trajectoryRadius < 0) {
    angle *= -1.0;
  }

  if (trajectoryType == TrajectoryType::CircleXY || trajectoryType == TrajectoryType::Circle3D) {
    CartesianPoint u = scalePoint(trajectoryU, trajectoryRadius * cos(angle));
    CartesianPoint v = scalePoint(trajectoryV, trajectoryRadius * sin(angle));
    point = addPoints(trajectoryCenter, addPoints(u, v));
    return true;
  }

  if (trajectoryType == TrajectoryType::Lissajous) {
    point.x = trajectoryCenter.x + lissajousAmplitude.x * sin(2 * PI * lissajousFrequency.x * t);
    point.y = trajectoryCenter.y + lissajousAmplitude.y * sin(2 * PI * lissajousFrequency.y * t + lissajousPhaseY);
    point.z = trajectoryCenter.z + lissajousAmplitude.z * sin(2 * PI * lissajousFrequency.z * t + lissajousPhaseZ);
    return true;
  }

  if (trajectoryType == TrajectoryType::Rose) {
    float radius = trajectoryRadius * cos(rosePetals * angle);
    CartesianPoint u = scalePoint(trajectoryU, radius * cos(angle));
    CartesianPoint v = scalePoint(trajectoryV, radius * sin(angle));
    point = addPoints(trajectoryCenter, addPoints(u, v));
    return true;
  }

  return false;
}

bool DeltaRobot::setupPlaneBasis(const CartesianPoint& normal) {
  CartesianPoint n = normal;
  if (!normalizePoint(n)) return false;

  CartesianPoint reference = fabs(n.z) < 0.9 ? makePoint(0, 0, 1) : makePoint(0, 1, 0);
  trajectoryU = crossProduct(reference, n);
  if (!normalizePoint(trajectoryU)) return false;
  trajectoryV = crossProduct(n, trajectoryU);
  return normalizePoint(trajectoryV);
}

DeltaRobot::CartesianPoint DeltaRobot::makePoint(float x, float y, float z) const {
  CartesianPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

DeltaRobot::CartesianPoint DeltaRobot::addPoints(const CartesianPoint& a, const CartesianPoint& b) const {
  return makePoint(a.x + b.x, a.y + b.y, a.z + b.z);
}

DeltaRobot::CartesianPoint DeltaRobot::scalePoint(const CartesianPoint& point, float scale) const {
  return makePoint(point.x * scale, point.y * scale, point.z * scale);
}

DeltaRobot::CartesianPoint DeltaRobot::crossProduct(const CartesianPoint& a, const CartesianPoint& b) const {
  return makePoint(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

float DeltaRobot::dotProduct(const CartesianPoint& a, const CartesianPoint& b) const {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

float DeltaRobot::vectorLength(const CartesianPoint& point) const {
  return sqrt(dotProduct(point, point));
}

bool DeltaRobot::normalizePoint(CartesianPoint& point) const {
  float length = vectorLength(point);
  if (length < 0.0001) return false;
  point.x /= length;
  point.y /= length;
  point.z /= length;
  return true;
}

float DeltaRobot::circleApproachTime(float motionTime) const {
  float approachTime = motionTime * circleApproachPercent / 100.0;
  if (approachTime < circleApproachMinTime) approachTime = circleApproachMinTime;
  if (approachTime > circleApproachMaxTime) approachTime = circleApproachMaxTime;
  return approachTime;
}

float DeltaRobot::distanceBetween(const CartesianPoint& a, const CartesianPoint& b) const {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

void DeltaRobot::markPendingPosition(const CartesianPoint& target) {
  pendingPosition = target;
  pendingPositionValid = true;
}

void DeltaRobot::clearPendingPosition() {
  pendingPositionValid = false;
}

void DeltaRobot::prepareInterpolation(float target[], float motion_time) {
  motionMode = MotionMode::JointInterpolation;
  totalSteps = motionTimeToSteps(motion_time);
  currentStep = 0;
  for (int i = 0; i < 3; i++) stepAngles[i] = (target[i] - currentAngles[i]) / totalSteps;
}

void DeltaRobot::debugPrintAngles() {
  Serial.printf("[%s] Target Angles: %.2f %.2f %.2f\n", _tag, targetAngles[0], targetAngles[1], targetAngles[2]);
}

void DeltaRobot::stopServos() {
  Serial.printf("[%s] Stopping Servos at: %.2f %.2f %.2f\n", _tag, currentAngles[0], currentAngles[1], currentAngles[2]);
  for (int i = 0; i < 3; i++) setServoAngle(i, currentAngles[i]);
}

void DeltaRobot::setServoAngle(int channel, float angle) {
  // 与原版一致：0..180 钳位后换算脉宽；输出到本侧物理通道
  _servo->setAngle(_physOffset + channel, constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX));
  currentAngles[channel] = angle;
}

int DeltaRobot::angleToPulse(float angle) {
  return (int)(SERVO_PULSE_MIN_US + (angle / 180.0) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US));
}

int DeltaRobot::motionTimeToSteps(float motionTime) const {
  return max(1, (int)(motionTime / MOTION_TICK_MS));
}

DeltaRobot::CartesianPoint DeltaRobot::percentXYZToXYZ(float percent_x, float percent_y, float percent_z)
{
  CartesianPoint point;
  point.x = homeX + percent_x / 100.0 * workspaceRadius;
  point.y = homeY + percent_y / 100.0 * workspaceRadius;
  point.z = homeZ + percent_z / 100.0 * workspaceHeight;
  return point;
}

float DeltaRobot::percentRadiusToRadius(float percent_r)
{
  return percent_r / 100.0 * workspaceRadius;
}

int DeltaRobot::inverseKinematics(float x0, float y0, float z0, float angles[]) {
  float theta1, theta2, theta3;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0 * cos(120 * PI / 180) + y0 * sin(120 * PI / 180), y0 * cos(120 * PI / 180) - x0 * sin(120 * PI / 180), z0, theta2);
  if (status == 0) status = delta_calcAngleYZ(x0 * cos(120 * PI / 180) - y0 * sin(120 * PI / 180), y0 * cos(120 * PI / 180) + x0 * sin(120 * PI / 180), z0, theta3);
  angles[0] = -theta1 + 90;
  angles[1] = -theta2 + 90;
  angles[2] = -theta3 + 90;
  return status;
}

bool DeltaRobot::forwardKinematics(float a1, float a2, float a3, CartesianPoint& point) const {
  float theta1 = (90.0 - a1) * PI / 180.0;
  float theta2 = (90.0 - a2) * PI / 180.0;
  float theta3 = (90.0 - a3) * PI / 180.0;
  float t = baseRadius - effectorRadius;

  float y1 = -(t + bicepLength * cos(theta1));
  float z1 = -bicepLength * sin(theta1);

  float y2 = (t + bicepLength * cos(theta2)) * sin(30 * PI / 180.0);
  float x2 = y2 * tan(60 * PI / 180.0);
  float z2 = -bicepLength * sin(theta2);

  float y3 = (t + bicepLength * cos(theta3)) * sin(30 * PI / 180.0);
  float x3 = -y3 * tan(60 * PI / 180.0);
  float z3 = -bicepLength * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  if (fabs(dnm) < 0.0001) return false;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  float a1c = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1c = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
  float a2c = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2c = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  float qa = a1c * a1c + a2c * a2c + dnm * dnm;
  float qb = 2.0 * (a1c * b1c + a2c * (b2c - y1 * dnm) - z1 * dnm * dnm);
  float qc = (b2c - y1 * dnm) * (b2c - y1 * dnm) + b1c * b1c + dnm * dnm * (z1 * z1 - forearmLength * forearmLength);
  float discriminant = qb * qb - 4.0 * qa * qc;
  if (discriminant < 0.0) return false;

  point.z = -0.5 * (qb + sqrt(discriminant)) / qa;
  point.x = (a1c * point.z + b1c) / dnm;
  point.y = (a2c * point.z + b2c) / dnm;
  return true;
}

bool DeltaRobot::markPendingPositionFromJointAngles(float a1, float a2, float a3) {
  CartesianPoint point;
  if (!forwardKinematics(a1, a2, a3, point)) return false;
  if (!isWithinWorkspace(point.x, point.y, point.z)) return false;

  markPendingPosition(point);
  return true;
}

int DeltaRobot::delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -baseRadius;
  y0 -= effectorRadius;
  float rf = bicepLength;
  float re = forearmLength;
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0) return -1;
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1);
  float zj = a + b * yj;
  theta = atan(-zj / (y1 - yj)) * 180 / PI;
  return 0;
}

bool DeltaRobot::isWithinWorkspace(float x, float y, float z) {
  float dx = x - homeX;
  float dy = y - homeY;
  float dz = z - homeZ;
  return (sqrt(dx * dx + dy * dy) <= workspaceRadius + 0.1 && fabs(dz) <= workspaceHeight + 0.1);
}

void DeltaRobot::addRepeatCommand(const String& cmd) {
  if (commandCount < MAX_REPEAT_COMMANDS) {
    commandList[commandCount++] = cmd;
  }
}

void DeltaRobot::selectAndRunRandomCommand() {
  if (commandCount == 0) return;
  String selectedCommand = commandList[random(0, commandCount)];
  processInternalCommand(selectedCommand);
  Serial.printf("[%s] Selected: %s\n", _tag, selectedCommand.c_str());
}

void DeltaRobot::setRepeatCommands(const String commands[], int count) {
  commandCount = 0;
  int maxCommands = min(count, MAX_REPEAT_COMMANDS);

  for (int i = 0; i < maxCommands; i++) {
    commandList[i] = commands[i];
    commandCount++;
  }

  Serial.printf("[%s] Repeat commands set: %d\n", _tag, commandCount);
  if (isRandomRepeat) randomSwitchTime = millis();
}

void DeltaRobot::processInternalCommand(const String& command) {
  ParsedCommand parsed;
  if (!parseCommand(command.c_str(), parsed)) return;
  if (parsed.type != CommandType::Battery) clearActiveMotion();
  executeCommand(parsed);
}

// ============================================================
// T-Code（本侧独立）
// ============================================================

void DeltaRobot::applyTCodeAxes(const ParsedTCodeCommand& command) {
  float motionTime = tcodeDefaultMotionTime;
  bool hasMappedLinear = false;
  for (int i = 0; i < 3; i++) {
    int channel = tcodeAxisChannel[i];
    if (channel < 0 || channel >= TCODE_CHANNEL_COUNT) continue;

    const TCodeAxisCommand* axisCommand = findTCodeAxisCommand(command, tcodeAxisType[i], channel);
    if (axisCommand == nullptr || !axisCommand->active) continue;

    hasMappedLinear = true;
    float axisTime = tcodeMotionTimeForAxis(i, *axisCommand);
    if (axisTime > motionTime) motionTime = axisTime;
    tcodeLinearValues[i] = axisCommand->magnitude;
  }

  if (!hasMappedLinear) return;

  CartesianPoint target = tcodeTargetFromState();
  startTCodeLiveMotion(target, motionTime);
}

void DeltaRobot::tcodeTimeoutStop() {
  if (motionMode == MotionMode::TCodeLive) stopTCodeLiveMotion(false);
}

void DeltaRobot::tcodeStop() {
  if (motionMode == MotionMode::TCodeLive) stopTCodeLiveMotion(false);
  clearActiveMotion();
  isRandomRepeat = false;
}

bool DeltaRobot::isTCodeLive() const {
  return motionMode == MotionMode::TCodeLive;
}

float DeltaRobot::tcodeMotionTimeForAxis(int axis, const TCodeAxisCommand& command) const {
  if (command.hasInterval) {
    float interval = command.intervalMs;
    if (interval < tcodeMinMotionTime) interval = tcodeMinMotionTime;
    if (interval > tcodeMaxMotionTime) interval = tcodeMaxMotionTime;
    return interval;
  }

  if (command.hasSpeed && command.speedPerSecond > 0.0001f && axis >= 0 && axis < 3) {
    float distance = fabs(command.magnitude - tcodeLinearValues[axis]);
    float interval = distance / command.speedPerSecond * 1000.0f;
    if (interval < tcodeMinMotionTime) interval = tcodeMinMotionTime;
    if (interval > tcodeMaxMotionTime) interval = tcodeMaxMotionTime;
    return interval;
  }

  return tcodeDefaultMotionTime;
}

float DeltaRobot::tcodeMagnitudeToPercent(int axis, float magnitude) const {
  if (magnitude < 0.0f) magnitude = 0.0f;
  if (magnitude > 0.9999f) magnitude = 0.9999f;
  if (axis >= 0 && axis < 3 && tcodeAxisInvert[axis]) magnitude = 0.9999f - magnitude;

  float minPercent = axis >= 0 && axis < 3 ? tcodeAxisMinPercent[axis] : -100.0f;
  float maxPercent = axis >= 0 && axis < 3 ? tcodeAxisMaxPercent[axis] : 100.0f;
  return minPercent + magnitude * (maxPercent - minPercent);
}

DeltaRobot::CartesianPoint DeltaRobot::tcodeTargetFromState() {
  float percent[3] = {0.0f, 0.0f, 0.0f};
  for (int axis = 0; axis < 3; axis++) {
    int coord = tcodeAxisCoord[axis];
    if (coord >= 0 && coord < 3) percent[coord] = tcodeMagnitudeToPercent(axis, tcodeLinearValues[axis]);
  }

  return percentXYZToXYZ(percent[0], percent[1], percent[2]);
}

void DeltaRobot::startTCodeLiveMotion(const CartesianPoint& target, float motionTime) {
  if (!isWithinWorkspace(target.x, target.y, target.z)) return;
  if (inverseKinematics(target.x, target.y, target.z, targetAngles) != 0) return;

  if (!currentPositionValid) {
    currentPosition = makePoint(homeX, homeY, homeZ);
    currentPositionValid = true;
  }

  if (motionMode != MotionMode::TCodeLive) {
    clearActiveMotion();
    motionMode = MotionMode::TCodeLive;
  }

  tcodeLiveStartPosition = currentPosition;
  tcodeLiveTargetPosition = target;
  tcodeLiveTotalSteps = motionTimeToSteps(motionTime);
  tcodeLiveCurrentStep = 0;
}

void DeltaRobot::stopTCodeLiveMotion(bool stopVibration) {
  if (motionMode == MotionMode::TCodeLive) {
    motionMode = MotionMode::Idle;
    currentStep = totalSteps;
  }

  if (stopVibration && _vibratorSetter) _vibratorSetter(0, false);
}

const TCodeAxisCommand* DeltaRobot::findTCodeAxisCommand(const ParsedTCodeCommand& command, char axisType, int channel) const {
  for (int i = 0; i < command.axisCount; i++) {
    if (command.axes[i].axisType == axisType && command.axes[i].channel == channel) {
      return &command.axes[i].command;
    }
  }
  return nullptr;
}

// ============================================================
// T-Code 轴槽位（本侧 3 槽位）
// ============================================================

void DeltaRobot::resetAxisSlots() {
  tcodeAxisType[0] = 'L';
  tcodeAxisType[1] = 'L';
  tcodeAxisType[2] = 'L';
  tcodeAxisChannel[0] = 0;
  tcodeAxisChannel[1] = 1;
  tcodeAxisChannel[2] = 2;
  tcodeAxisCoord[0] = 0;
  tcodeAxisCoord[1] = 2;
  tcodeAxisCoord[2] = 1;

  for (int i = 0; i < 3; i++) {
    tcodeAxisInvert[i] = false;
    tcodeAxisMinPercent[i] = -100.0f;
    tcodeAxisMaxPercent[i] = 100.0f;
  }
}

void DeltaRobot::setAxisSlot(int slot, char axisType, int channel, int coord,
                             bool invert, float minPercent, float maxPercent) {
  if (slot < 0 || slot >= 3) return;
  tcodeAxisType[slot] = axisType;
  tcodeAxisChannel[slot] = channel;
  tcodeAxisCoord[slot] = coord;
  tcodeAxisInvert[slot] = invert;
  tcodeAxisMinPercent[slot] = minPercent;
  tcodeAxisMaxPercent[slot] = maxPercent;
}

void DeltaRobot::getAxisSlot(int slot, char& axisType, int& channel, int& coord,
                             bool& invert, float& minPercent, float& maxPercent) const {
  if (slot < 0 || slot >= 3) return;
  axisType = tcodeAxisType[slot];
  channel = tcodeAxisChannel[slot];
  coord = tcodeAxisCoord[slot];
  invert = tcodeAxisInvert[slot];
  minPercent = tcodeAxisMinPercent[slot];
  maxPercent = tcodeAxisMaxPercent[slot];
}

String DeltaRobot::axisLabel(int slot) const {
  if (slot < 0 || slot >= 3) return "?";
  String label = "";
  label += tcodeAxisType[slot];
  label += String(tcodeAxisChannel[slot]);
  return label;
}

const char* DeltaRobot::coordName(int coord) const {
  if (coord == 0) return "X";
  if (coord == 1) return "Y";
  if (coord == 2) return "Z";
  return "?";
}
