// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Ported unchanged; only the header comment and platform includes were adjusted.

#ifndef DELTA_COMMAND_PARSER_H
#define DELTA_COMMAND_PARSER_H

// ============================================================
// 原版 CDR01 固件的命令解析器（原样移植，协议层零改动）
// 来源: 0721robotics/cdr01-firmware DeltaCommandParser.h (Apache-2.0)
// ============================================================

#include <stdint.h>

static constexpr int TCODE_CHANNEL_COUNT = 10;
static constexpr int TCODE_MAX_AXIS_COMMANDS = 24;

enum class CommandType : uint8_t {
  Unknown,
  Point,
  Line,
  Joint,
  Circle,
  Circle3D,
  Lissajous,
  Rose,
  RepeatPoint,
  RepeatLine,
  RepeatTriangle,
  RepeatCircle,
  RepeatCircle3D,
  RepeatLissajous,
  RepeatRose,
  RepeatRandom,
  Vibrate,
  Battery,
  Stop,
};

struct ParsedCommand {
  CommandType type;
  float values[16];
  int valueCount;
  int intValue;
};

struct TCodeAxisCommand {
  bool active;
  float magnitude;
  bool hasInterval;
  int intervalMs;
  bool hasSpeed;
  float speedPerSecond;
};

struct TCodeParsedAxis {
  char axisType;
  int channel;
  TCodeAxisCommand command;
};

struct ParsedTCodeCommand {
  TCodeParsedAxis axes[TCODE_MAX_AXIS_COMMANDS];
  int axisCount;
  TCodeAxisCommand linear[TCODE_CHANNEL_COUNT];
  TCodeAxisCommand vibrate[TCODE_CHANNEL_COUNT];
  bool hasAxis;
  bool hasLinear;
  bool hasRotation;
  bool hasVibrate;
  bool stop;
  bool saveCommand;
  int deviceCommand;
};

CommandType identifyCommand(const char* command);
const char* commandTypeName(CommandType type);
bool parseCommand(const char* command, ParsedCommand& parsed);
bool looksLikeTCodeCommand(const char* command);
bool parseTCodeCommand(const char* command, ParsedTCodeCommand& parsed);

#endif
