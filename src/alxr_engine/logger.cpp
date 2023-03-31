// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "pch.h"
#include "logger.h"

#include <atomic>
#include <cassert>
#include <array>
#include <string_view>
#include <sstream>

#if defined(ANDROID)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, "alxr-client", __VA_ARGS__)
#define ALOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, "alxr-client", __VA_ARGS__)
#endif

namespace {;

inline void defaultOuput(Log::Level severity, const char* output, unsigned int len) {
    assert(output != nullptr);
    
    const std::string_view out{ output, len };

    ((severity == Log::Level::Error) ? std::clog : std::cout) << out;
#if defined(_WIN32)
    OutputDebugStringA(output);
#endif
#if defined(ANDROID)
    if (severity == Log::Level::Error)
        ALOGE("%s", output);
    else
        ALOGV("%s", output);
#endif
}

std::atomic<Log::OutputFn> g_outputFn{ defaultOuput };
std::atomic<unsigned int>  g_logOptions { Log::LogOptions::Timestamp | Log::LogOptions::LevelTag };
Log::Level g_minSeverity{Log::Level::Info};
std::mutex g_logLock;

}  // namespace

namespace Log {
void SetLevel(Level minSeverity) { g_minSeverity = minSeverity; }

void Write(Level severity, const std::string& msg) {
    if (severity < g_minSeverity) {
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const time_t now_time = std::chrono::system_clock::to_time_t(now);
    tm now_tm;
#ifdef _WIN32
    localtime_s(&now_tm, &now_time);
#else
    localtime_r(&now_time, &now_tm);
#endif
    // time_t only has second precision. Use the rounding error to get sub-second precision.
    const auto secondRemainder = now - std::chrono::system_clock::from_time_t(now_time);
    const int64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(secondRemainder).count();

    // { Verbose, Info, Warning, Error };
    static constexpr const std::array<std::string_view, 4> severityName {
        "Verbose", "Info", "Warning", "Error"
    };

    const auto logOpts = g_logOptions.load();
    const auto outputFn = g_outputFn.load();
    assert(outputFn != nullptr);

    if (logOpts == 0) {
        std::scoped_lock<std::mutex> lock(g_logLock);  // Ensure output is serialized    
        outputFn(severity, msg.c_str(), static_cast<unsigned int>(msg.length()));
        return;
    }

    std::ostringstream out;
    out.fill('0');
    if ((logOpts & LogOptions::Timestamp) != 0) {
        out << "[" << std::setw(2) << now_tm.tm_hour << ":" << std::setw(2) << now_tm.tm_min << ":" << std::setw(2) << now_tm.tm_sec
            << "." << std::setw(3) << milliseconds << "]";
    }
    if ((logOpts & LogOptions::LevelTag) != 0) {
        out << "[" << severityName[static_cast<std::size_t>(severity)] << "] ";
    }
    out << msg << std::endl;

    std::scoped_lock<std::mutex> lock(g_logLock);  // Ensure output is serialized    
    const auto& result = out.str();
    outputFn(severity, result.c_str(), static_cast<unsigned int>(result.length()));
}

void SetLogCustomOutput(const LogOptions options, OutputFn outputFn) {
    g_logOptions = options;
    if (outputFn == nullptr)
        outputFn = defaultOuput;
    assert(outputFn != nullptr);
    g_outputFn.store(outputFn);
}

}  // namespace Log
