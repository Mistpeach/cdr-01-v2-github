// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Mistpeach
//
// Derived from cdr01-firmware by 0721Robotics (Apache-2.0):
//   https://github.com/0721robotics/cdr01-firmware
// Ported unchanged; only the header comment and platform includes were adjusted.

#include "DeltaCommandParser.h"

// ============================================================
// 原版 CDR01 固件的命令解析器（原样移植，协议层零改动）
// 来源: 0721robotics/cdr01-firmware DeltaCommandParser.cpp (Apache-2.0)
// ============================================================

#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace {

const char* skipSpaces(const char* text) {
  while (text != nullptr && *text != '\0' && isspace((unsigned char)*text)) text++;
  return text;
}

bool startsWithToken(const char* text, const char* token) {
  text = skipSpaces(text);
  if (text == nullptr) return false;

  size_t tokenLength = strlen(token);
  if (strncmp(text, token, tokenLength) != 0) return false;

  char next = text[tokenLength];
  return next == '\0' || isspace((unsigned char)next);
}

bool startsWithTextIgnoreCase(const char* text, const char* token) {
  if (text == nullptr || token == nullptr) return false;
  while (*token != '\0') {
    if (toupper((unsigned char)*text) != toupper((unsigned char)*token)) return false;
    text++;
    token++;
  }
  return true;
}

void resetParsedCommand(ParsedCommand& parsed) {
  parsed.type = CommandType::Unknown;
  parsed.valueCount = 0;
  parsed.intValue = 0;
  for (int i = 0; i < 16; i++) parsed.values[i] = 0.0f;
}

void resetTCodeAxisCommand(TCodeAxisCommand& command) {
  command.active = false;
  command.magnitude = 0.0f;
  command.hasInterval = false;
  command.intervalMs = 0;
  command.hasSpeed = false;
  command.speedPerSecond = 0.0f;
}

void resetParsedTCodeCommand(ParsedTCodeCommand& parsed) {
  parsed.axisCount = 0;
  for (int i = 0; i < TCODE_MAX_AXIS_COMMANDS; i++) {
    parsed.axes[i].axisType = '\0';
    parsed.axes[i].channel = -1;
    resetTCodeAxisCommand(parsed.axes[i].command);
  }
  for (int i = 0; i < TCODE_CHANNEL_COUNT; i++) {
    resetTCodeAxisCommand(parsed.linear[i]);
    resetTCodeAxisCommand(parsed.vibrate[i]);
  }
  parsed.hasAxis = false;
  parsed.hasLinear = false;
  parsed.hasRotation = false;
  parsed.hasVibrate = false;
  parsed.stop = false;
  parsed.saveCommand = false;
  parsed.deviceCommand = -1;
}

bool storeParsedTCodeAxis(ParsedTCodeCommand& parsed, char axisType, int channel, const TCodeAxisCommand& command) {
  for (int i = 0; i < parsed.axisCount; i++) {
    if (parsed.axes[i].axisType == axisType && parsed.axes[i].channel == channel) {
      parsed.axes[i].command = command;
      return true;
    }
  }

  if (parsed.axisCount >= TCODE_MAX_AXIS_COMMANDS) return false;

  parsed.axes[parsed.axisCount].axisType = axisType;
  parsed.axes[parsed.axisCount].channel = channel;
  parsed.axes[parsed.axisCount].command = command;
  parsed.axisCount++;
  return true;
}

bool isTokenEnd(char c) {
  return c == '\0' || isspace((unsigned char)c);
}

bool parseUnsignedInt(const char*& text, int& value) {
  if (text == nullptr || !isdigit((unsigned char)*text)) return false;

  value = 0;
  while (isdigit((unsigned char)*text)) {
    value = value * 10 + (*text - '0');
    text++;
  }
  return true;
}

bool parseMagnitudeDigits(const char*& text, float& magnitude) {
  if (text == nullptr || !isdigit((unsigned char)*text)) return false;

  magnitude = 0.0f;
  float scale = 0.1f;
  while (isdigit((unsigned char)*text)) {
    magnitude += (*text - '0') * scale;
    scale *= 0.1f;
    text++;
  }
  if (magnitude >= 1.0f) magnitude = 0.9999f;
  return true;
}

bool parseTCodeAxisToken(const char*& text, ParsedTCodeCommand& parsed) {
  char axisType = (char)toupper((unsigned char)*text);
  if (axisType < 'A' || axisType > 'Z') return false;
  text++;

  int channel = *text - '0';
  if (channel < 0 || channel >= TCODE_CHANNEL_COUNT) return false;
  text++;

  TCodeAxisCommand axisCommand;
  resetTCodeAxisCommand(axisCommand);
  if (!parseMagnitudeDigits(text, axisCommand.magnitude)) return false;

  while (!isTokenEnd(*text)) {
    char modifier = (char)toupper((unsigned char)*text);
    text++;

    int value = 0;

    if (modifier == 'I') {
      if (!parseUnsignedInt(text, value)) return false;
      axisCommand.hasInterval = true;
      axisCommand.intervalMs = value;
    } else if (modifier == 'S') {
      float speed = 0.0f;
      if (!parseMagnitudeDigits(text, speed)) return false;
      axisCommand.hasSpeed = true;
      axisCommand.speedPerSecond = speed * 10.0f;
    } else {
      return false;
    }
  }

  axisCommand.active = true;
  if (!storeParsedTCodeAxis(parsed, axisType, channel, axisCommand)) return false;
  parsed.hasAxis = true;

  if (axisType == 'L') {
    parsed.linear[channel] = axisCommand;
    parsed.hasLinear = true;
    return true;
  }

  if (axisType == 'V') {
    parsed.vibrate[channel] = axisCommand;
    parsed.hasVibrate = true;
    return true;
  }

  if (axisType == 'R') {
    parsed.hasRotation = true;
    return true;
  }

  return true;
}

}  // namespace

CommandType identifyCommand(const char* command) {
  if (startsWithToken(command, "REPEAT_CIRCLE3D")) return CommandType::RepeatCircle3D;
  if (startsWithToken(command, "REPEAT_LISSAJOUS")) return CommandType::RepeatLissajous;
  if (startsWithToken(command, "REPEAT_TRIANGLE")) return CommandType::RepeatTriangle;
  if (startsWithToken(command, "REPEAT_ROSE")) return CommandType::RepeatRose;
  if (startsWithToken(command, "REPEAT_CIRCLE")) return CommandType::RepeatCircle;
  if (startsWithToken(command, "REPEAT_POINT")) return CommandType::RepeatPoint;
  if (startsWithToken(command, "REPEAT_LINE")) return CommandType::RepeatLine;
  if (startsWithToken(command, "REPEAT_RANDOM")) return CommandType::RepeatRandom;
  if (startsWithToken(command, "CIRCLE3D")) return CommandType::Circle3D;
  if (startsWithToken(command, "LISSAJOUS")) return CommandType::Lissajous;
  if (startsWithToken(command, "CIRCLE")) return CommandType::Circle;
  if (startsWithToken(command, "ROSE")) return CommandType::Rose;
  if (startsWithToken(command, "JOINT")) return CommandType::Joint;
  if (startsWithToken(command, "POINT")) return CommandType::Point;
  if (startsWithToken(command, "LINE")) return CommandType::Line;
  if (startsWithToken(command, "VIBRATE")) return CommandType::Vibrate;
  if (startsWithToken(command, "BATTERY")) return CommandType::Battery;
  if (startsWithToken(command, "STOP")) return CommandType::Stop;
  return CommandType::Unknown;
}

const char* commandTypeName(CommandType type) {
  switch (type) {
    case CommandType::Point: return "POINT";
    case CommandType::Line: return "LINE";
    case CommandType::Joint: return "JOINT";
    case CommandType::Circle: return "CIRCLE";
    case CommandType::Circle3D: return "CIRCLE3D";
    case CommandType::Lissajous: return "LISSAJOUS";
    case CommandType::Rose: return "ROSE";
    case CommandType::RepeatPoint: return "REPEAT_POINT";
    case CommandType::RepeatLine: return "REPEAT_LINE";
    case CommandType::RepeatTriangle: return "REPEAT_TRIANGLE";
    case CommandType::RepeatCircle: return "REPEAT_CIRCLE";
    case CommandType::RepeatCircle3D: return "REPEAT_CIRCLE3D";
    case CommandType::RepeatLissajous: return "REPEAT_LISSAJOUS";
    case CommandType::RepeatRose: return "REPEAT_ROSE";
    case CommandType::RepeatRandom: return "REPEAT_RANDOM";
    case CommandType::Vibrate: return "VIBRATE";
    case CommandType::Battery: return "BATTERY";
    case CommandType::Stop: return "STOP";
    default: return "UNKNOWN";
  }
}

bool parseCommand(const char* command, ParsedCommand& parsed) {
  resetParsedCommand(parsed);
  parsed.type = identifyCommand(command);

  switch (parsed.type) {
    case CommandType::Point:
      parsed.valueCount = sscanf(command, " POINT %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2], &parsed.values[3]);
      return parsed.valueCount == 4;

    case CommandType::Line:
      parsed.valueCount = sscanf(command, " LINE %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2], &parsed.values[3]);
      return parsed.valueCount == 4;

    case CommandType::Joint:
      parsed.valueCount = sscanf(command, " JOINT %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2], &parsed.values[3]);
      return parsed.valueCount == 4;

    case CommandType::Circle:
      parsed.valueCount = sscanf(command, " CIRCLE %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4]);
      return parsed.valueCount == 5;

    case CommandType::Circle3D:
      parsed.valueCount = sscanf(command, " CIRCLE3D %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7]);
      return parsed.valueCount == 8;

    case CommandType::Lissajous:
      parsed.valueCount = sscanf(command, " LISSAJOUS %f %f %f %f %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7], &parsed.values[8],
                                 &parsed.values[9], &parsed.values[10], &parsed.values[11]);
      return parsed.valueCount == 12;

    case CommandType::Rose:
      parsed.valueCount = sscanf(command, " ROSE %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5]);
      return parsed.valueCount == 6;

    case CommandType::RepeatPoint:
      parsed.valueCount = sscanf(command, " REPEAT_POINT %f %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7], &parsed.values[8]);
      return parsed.valueCount == 7 || parsed.valueCount == 9;

    case CommandType::RepeatLine:
      parsed.valueCount = sscanf(command, " REPEAT_LINE %f %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7], &parsed.values[8]);
      return parsed.valueCount == 7 || parsed.valueCount == 9;

    case CommandType::RepeatTriangle:
      parsed.valueCount = sscanf(command, " REPEAT_TRIANGLE %f %f %f %f %f %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7], &parsed.values[8],
                                 &parsed.values[9], &parsed.values[10], &parsed.values[11],
                                 &parsed.values[12]);
      return parsed.valueCount == 10 || parsed.valueCount == 13;

    case CommandType::RepeatCircle:
      parsed.valueCount = sscanf(command, " REPEAT_CIRCLE %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4]);
      return parsed.valueCount == 5;

    case CommandType::RepeatCircle3D:
      parsed.valueCount = sscanf(command, " REPEAT_CIRCLE3D %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7]);
      return parsed.valueCount == 8;

    case CommandType::RepeatLissajous:
      parsed.valueCount = sscanf(command, " REPEAT_LISSAJOUS %f %f %f %f %f %f %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5],
                                 &parsed.values[6], &parsed.values[7], &parsed.values[8],
                                 &parsed.values[9], &parsed.values[10], &parsed.values[11]);
      return parsed.valueCount == 12;

    case CommandType::RepeatRose:
      parsed.valueCount = sscanf(command, " REPEAT_ROSE %f %f %f %f %f %f",
                                 &parsed.values[0], &parsed.values[1], &parsed.values[2],
                                 &parsed.values[3], &parsed.values[4], &parsed.values[5]);
      return parsed.valueCount == 6;

    case CommandType::Vibrate:
      parsed.valueCount = sscanf(command, " VIBRATE %d", &parsed.intValue);
      return parsed.valueCount == 1;

    case CommandType::RepeatRandom:
    case CommandType::Battery:
    case CommandType::Stop:
      return true;

    default:
      return false;
  }
}

bool looksLikeTCodeCommand(const char* command) {
  const char* text = skipSpaces(command);
  if (text == nullptr || *text == '\0') return false;

  char first = (char)toupper((unsigned char)*text);
  if (first == '$') return true;
  if (first == 'D') {
    return isTokenEnd(text[1]) || isdigit((unsigned char)text[1]) || startsWithTextIgnoreCase(text, "DSTOP");
  }

  if (first >= 'A' && first <= 'Z') return isdigit((unsigned char)text[1]);
  return false;
}

bool parseTCodeCommand(const char* command, ParsedTCodeCommand& parsed) {
  resetParsedTCodeCommand(parsed);

  const char* text = skipSpaces(command);
  if (!looksLikeTCodeCommand(text)) return false;

  while (text != nullptr && *text != '\0') {
    text = skipSpaces(text);
    if (*text == '\0') break;

    char first = (char)toupper((unsigned char)*text);
    if (first == '$') {
      parsed.saveCommand = true;
      while (!isTokenEnd(*text)) text++;
    } else if (first == 'D') {
      if (startsWithTextIgnoreCase(text, "DSTOP")) {
        text += 5;
        if (!isTokenEnd(*text)) return false;
        parsed.stop = true;
      } else {
        text++;
        int deviceCommand = 0;
        if (!parseUnsignedInt(text, deviceCommand)) return false;
        parsed.deviceCommand = deviceCommand;
      }
    } else if (first >= 'A' && first <= 'Z' && isdigit((unsigned char)text[1])) {
      if (!parseTCodeAxisToken(text, parsed)) return false;
    } else {
      return false;
    }

    if (!isTokenEnd(*text)) return false;
  }

  return parsed.hasAxis || parsed.hasLinear || parsed.hasRotation || parsed.hasVibrate || parsed.stop || parsed.saveCommand || parsed.deviceCommand >= 0;
}
