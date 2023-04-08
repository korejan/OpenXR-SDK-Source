#pragma once
#ifndef ALXR_TIMING_H
#define ALXR_TIMING_H

#include <cstdint>
#include <ctime>
#include <chrono>
#include <sstream>
#include "logger.h"

#if 1 //def XR_USE_PLATFORM_WIN32
using XrSteadyClock = std::chrono::steady_clock;
#else
struct XrSteadyClock : std::chrono::steady_clock
{
    using time_point = std::chrono::time_point<XrSteadyClock, duration>;    
    constexpr static const bool is_steady = true;

    static inline time_point now() noexcept
    {
        using namespace std::chrono;
        struct timespec ts;
        clock_gettime(CLOCK_BOOTTIME, &ts);
        return time_point(seconds(ts.tv_sec) + nanoseconds(ts.tv_nsec));
    }
};
#endif

template < typename ClockType >
inline std::uint64_t GetTimestampUs()
{
    using namespace std::chrono;
    using microsecondsU64 = duration<std::uint64_t, microseconds::period>;
    return duration_cast<microsecondsU64>(ClockType::now().time_since_epoch()).count();
}

template < typename ClockType >
inline std::uint64_t GetTimestampMs()
{
    using namespace std::chrono;
    using millisecondsU64 = duration<std::uint64_t, milliseconds::period>;
    return duration_cast<millisecondsU64>(ClockType::now().time_since_epoch()).count();
}

// monotonic clock.
inline std::uint64_t GetSteadyTimestampUs()
{
    static_assert(XrSteadyClock::is_steady);
    return GetTimestampUs<XrSteadyClock>();
}

inline std::uint64_t GetSteadyTimestampMs()
{
    static_assert(XrSteadyClock::is_steady);
    return GetTimestampMs<XrSteadyClock>();
}

inline std::uint64_t GetSystemTimestampUs()
{
    return GetTimestampUs<std::chrono::system_clock>();
}

template < const bool enable, typename Fun >
inline decltype(auto) time_call_ms(Fun&& fn)
{
    using ClockType = XrSteadyClock;
    static_assert(ClockType::is_steady);
    using millisecondsf = std::chrono::duration<float, std::chrono::milliseconds::period>;

    if constexpr (enable)
    {
        using namespace std::chrono;
        struct ScopedTimer
        {
            using time_point = ClockType::time_point;
            millisecondsf::rep& time_in_ms;
            const time_point start = ClockType::now();

            constexpr inline ScopedTimer(millisecondsf::rep& t) : time_in_ms(t) {}

            inline ~ScopedTimer()
            {
                using namespace std::chrono;
                const auto end = ClockType::now();
                time_in_ms = duration_cast<millisecondsf>(end - start).count();
            }
        };
        if constexpr (std::is_void_v<decltype(fn())>)
        {
            millisecondsf::rep time_in_ms;
            {
                ScopedTimer scoped_timer(time_in_ms);
                fn();
            }
            return time_in_ms;
        }
        else
        {
            millisecondsf::rep time_in_ms;
            return std::make_tuple([&]()
            {
                ScopedTimer scoped_timer(time_in_ms);
                return fn();
            }(), time_in_ms);
        }
    }
    else if constexpr (std::is_void_v<decltype(fn())>)
    {
        fn();
        return millisecondsf::rep(0);
    }
    else
    {
        return std::make_tuple(fn(), millisecondsf::rep(0));
    }
}

template < const bool enable, typename Fun >
inline decltype(auto) time_call_ms(const char* name, Fun&& fn)
{
    if constexpr (std::is_void_v<decltype(fn())>)
    {
        auto t = time_call_ms<enable>(std::forward<Fun>(fn));
        if constexpr (enable) {
            std::ostringstream oss;
            oss << name << " took " << t << " ms\n";
            const auto val = oss.str();
            Log::Write(Log::Level::Info, val.c_str());
        }
        return;
    }
    else
    {
        auto&& [ret, t] = time_call_ms<enable>(std::forward<Fun>(fn));
        if constexpr (enable) {
            std::ostringstream oss;
            oss << name << " took " << t << " ms\n";
            const auto val = oss.str();
            Log::Write(Log::Level::Info, val.c_str());
        }
        return ret;
    }
}

#endif
