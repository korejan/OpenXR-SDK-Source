// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

namespace Log {
enum class Level : unsigned int { Verbose, Info, Warning, Error };

void SetLevel(Level minSeverity);
void Write(Level severity, const std::string& msg);

enum LogOptions : unsigned int {
	None = 0,
	Timestamp = (1<<0u),
	LevelTag  = (1<<1u)
};
using OutputFn = void (*)(Level level, const char* output, unsigned int len);
void SetLogCustomOutput(const LogOptions options, OutputFn outputFn);
}  // namespace Log
