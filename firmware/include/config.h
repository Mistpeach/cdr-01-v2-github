// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach

#pragma once
// ============================================================
// config.h —— CDR01 移植固件全部可调配置（中文注释）
// 改这里，不要改 src/ 下的业务代码。
// ============================================================

// ============================================================
// 0. 固件标识
// ============================================================
#define CDR01_FIRMWARE_VERSION "0.1.0"     // VERSION 命令与 T-Code D0 的返回值
#define CDR01_SKETCH_NAME "CDR01v2-ESP32S3"

// ============================================================
// 1. 舵机（6 个 SG90，6 个独立逻辑通道，左右两侧各 3 个）
// ============================================================
// 与原版的对应关系：
//   原版 = 左右两台独立设备，各用 3 通道（Motion Base S1/S2/S3），
//   由上位机双槽位（或 T-Code 分轴）分别控制，互不影响。
//   本板 = 一台 ESP32-S3 直驱全部 6 个舵机，逻辑通道扩展为 6 个：
//     通道 0..2 = 左侧（L0/L1/L2，与原版左机 S1/S2/S3 等价）
//     通道 3..5 = 右侧（R0/R1/R2，与原版右机 S1/S2/S3 等价）
//   每侧内部臂序约定与原版一致：
//     臂 0 = 俯视 +Y 方向，臂 1 = 120°，臂 2 = 240°
#define SERVO_COUNT 6                 // 物理通道总数（6 舵机）

#define SERVO_GPIO {4, 5, 15, 16, 17, 18}   // L0 L1 L2 | R0 R1 R2
#define SERVO_INVERT {false, false, false, false, false, false}
// 反相说明：某通道方向反了就把对应位置改 true（等效 pulse(180-angle)，无需改线）。

#define SERVO_PWM_FREQ 50             // Hz，20ms 周期（标准舵机）
#define SERVO_PWM_RESOLUTION 14       // bit。ESP32-S3 LEDC 最大 14 位（1.22µs/步，SG90 足够）
#define SERVO_PULSE_MIN_US 500        // 0° 脉宽（µs）。SG90 可收紧到 500
#define SERVO_PULSE_MAX_US 2500       // 180° 脉宽（µs）。SG90 可收紧到 2400
#define SERVO_ANGLE_MIN 0.0f          // 角度软限位（度）。保守可设 10
#define SERVO_ANGLE_MAX 180.0f        // 角度软限位（度）。保守可设 170

// ============================================================
// 2. Delta 几何常量（机械结构与原版一致 → 默认值与原版相同，勿改）
// ============================================================
#define DELTA_BASE_RADIUS 26.0f       // 底座臂轴分布圆半径 mm
#define DELTA_BICEP_LENGTH 25.0f      // 主动臂长 mm
#define DELTA_FOREARM_LENGTH 40.0f    // 从动臂长 mm
#define DELTA_EFFECTOR_RADIUS 16.0f   // 末端平台半径 mm
#define DELTA_HOME_X 0.0f
#define DELTA_HOME_Y 0.0f
#define DELTA_HOME_Z -33.0f           // Home 点 Z
#define DELTA_WORKSPACE_RADIUS 20.0f  // 工作空间半径 mm
#define DELTA_WORKSPACE_HEIGHT 15.0f  // 工作空间半高 mm

// ============================================================
// 3. 动作引擎时序（与原版一致，勿改）
// ============================================================
#define MOTION_TICK_MS 10             // 步进节拍 ms（原版硬件定时器 10ms）
#define TCODE_DEFAULT_MOTION_MS 50.0f // T-Code 无时间指定时的默认移动时间
#define TCODE_MIN_MOTION_MS 10.0f     // T-Code 移动时间下限
#define TCODE_MAX_MOTION_MS 5000.0f   // T-Code 移动时间上限
#define TCODE_TIMEOUT_MS 750          // T-Code 连续收不到数据即停止（安全超时）
#define CIRCLE_APPROACH_PERCENT 15.0f // 进轨 Approach 时间 = 动作时间的 15%
#define CIRCLE_APPROACH_MIN_MS 100.0f
#define CIRCLE_APPROACH_MAX_MS 1000.0f
#define RANDOM_TIME_MIN_MS 5000       // REPEAT_RANDOM 切换间隔范围
#define RANDOM_TIME_MAX_MS 20000

// ============================================================
// 4. 安全策略
// ============================================================
// BLE 断开时是否急停。false = 断线继续跑（适合单机自嗨）；
// true = 断开即 STOP（上位机连动时更安全）。
#define SAFETY_STOP_ON_BLE_DISCONNECT false
// 上电自检时是否小幅摆臂确认（false = 仅回中锁定，更安静）
#define SELF_TEST_SWING false

// ============================================================
// 5. 串口（调试通道，文本命令，支持 L:/R: 前缀）
// ============================================================
#define SERIAL_BAUD 115200

// ============================================================
// 6. BLE（v2 全新协议，见 ../01-通信协议规范.md）
// ============================================================
#define BLE_DEVICE_NAME "CDR01v2"
#define BLE_SERVICE_UUID "b4a00001-3f21-4c8e-9d5a-6b7e0f1a2c3d"
#define BLE_CMD_CHARACTERISTIC_UUID "b4a00002-3f21-4c8e-9d5a-6b7e0f1a2c3d"
#define BLE_EVT_CHARACTERISTIC_UUID "b4a00003-3f21-4c8e-9d5a-6b7e0f1a2c3d"
#define BLE_MTU 517                   // 服务端默认 MTU；上位机会再请求 ≥247
#define BLE_BATTERY_SERVICE_UUID 0x180F
#define BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID 0x2A19

// ---- V2 协议常量（与 01-通信协议规范.md 一致）----
#define V2_MAGIC_0 0xCD
#define V2_MAGIC_1 0x01
#define V2_SIDE_BOTH 0x00
#define V2_SIDE_LEFT 0x01
#define V2_SIDE_RIGHT 0x02
#define V2_EVT_RESPONSE 0x01
#define V2_EVT_HEARTBEAT 0x02
#define V2_MAX_PAYLOAD 240            // 单帧 payload 上限（原版最长命令 ~60B）
#define V2_HEARTBEAT_INTERVAL_MS 1000 // 连接期间心跳周期
#define V2_RX_BUFFER 520              // 接收缓冲（抗分片，> 最长帧 246B 的 2 倍）

// ============================================================
// 7. 振动电机（原版 GPIO8，PWM 1kHz/8bit）
// ============================================================
#define VIBRATOR_PIN 8                // 没有振动电机也保持此默认，输出 PWM 无副作用；-1 禁用
#define VIBRATOR_PWM_FREQ 1000
#define VIBRATOR_PWM_RESOLUTION 8

// ============================================================
// 8. 状态 LED
// ============================================================
// 原版在 GPIO35（N16R8 上被八线 PSRAM 占用），必须换脚。
// 类型：0=关闭  1=普通 GPIO LED  2=WS2812 灯珠（内置驱动，无需 FastLED）
#define LED_TYPE 1
#define LED_PIN 2                     // 普通 LED 用 GPIO2（板载 LED 常见位置）
#define LED_GPIO_ACTIVE_HIGH true     // 你的 LED 是高电平点亮还是低电平点亮
#define LED_WS2812_PIN 48             // WS2812 模式时的引脚（多数 N16R8 板 GPIO48 有板载 RGB）
#define LED_MAX_BRIGHTNESS 15         // 原版 FastLED 亮度 15（0-255），WS2812 模式生效

// ============================================================
// 9. 模式按键（原版 = AtomS3 面板键；单按循环 4 模式，双击回中）
// ============================================================
// -1 = 禁用。启用建议：0（板载 BOOT 键，注意 GPIO0 是 strapping 脚，
// 上电瞬间必须松开，运行中按没问题）或任意空闲脚（内部上拉，按键对地）。
#define BUTTON_PIN -1
#define BUTTON_DEBOUNCE_MS 50         // 消抖
#define BUTTON_DOUBLE_CLICK_MS 300    // 双击判定窗口

// ============================================================
// 10. 电池监测（原版 INA226，本板无此芯片）
// ============================================================
// 0 = 固定值（USB/外接电源供电场景，默认）
// 1 = ADC 分压测量（需接分压电阻）
#define BATTERY_SOURCE 0
#define BATTERY_FIXED_VOLTAGE 5.0f    // 固定模式返回值
#define BATTERY_FIXED_CURRENT 0.1f    // 固定模式返回值（>0 = 未充电，LED 不显示充电绿）
#define BATTERY_FIXED_LEVEL 100       // 固定模式电量百分比
#define BATTERY_ADC_PIN 14            // ADC 模式引脚（建议 ADC1 脚 GPIO1/2/6/7/8/9/10）
#define BATTERY_ADC_DIVIDER 2.0f      // 分压比 = (R1+R2)/R2，例 100k+100k = 2.0
#define BATTERY_ADC_VREF 3.3f         // ADC 满量程电压
#define BATTERY_EMPTY_VOLTAGE 3.17f   // 电量 0% 电压（与原版一致）
#define BATTERY_FULL_VOLTAGE 4.17f    // 电量 100% 电压（与原版一致）
