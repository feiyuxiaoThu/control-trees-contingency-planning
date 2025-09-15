/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:17:35
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 16:06:50
 * @FilePath: /dive-into-contingency-planning/common/common.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved. 
 */

#pragma once

#include "foxglove/time.hpp"
#include "foxglove/schemas.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <spdlog/spdlog.h>

#define LOG_TRACE(logger, ...)    SPDLOG_LOGGER_CALL(logger, spdlog::level::trace, __VA_ARGS__)
#define LOG_DEBUG(logger, ...)    SPDLOG_LOGGER_CALL(logger, spdlog::level::debug, __VA_ARGS__)
#define LOG_INFO(logger, ...)     SPDLOG_LOGGER_CALL(logger, spdlog::level::info,  __VA_ARGS__)
#define LOG_WARN(logger, ...)     SPDLOG_LOGGER_CALL(logger, spdlog::level::warn,  __VA_ARGS__)
#define LOG_ERROR(logger, ...)    SPDLOG_LOGGER_CALL(logger, spdlog::level::err,   __VA_ARGS__)
#define LOG_CRITICAL(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::critical, __VA_ARGS__)


struct State {
    double x{0};
    double y{0};
    double yaw{0};
    double velocity{0};

    foxglove::schemas::Pose to_pose() const {
        foxglove::schemas::Pose pose;
        pose.position = foxglove::schemas::Vector3{x, y, 0.0};
        pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, sin(yaw * 0.5), cos(yaw * 0.5)};
        return pose;
    }
};

struct Control {
    double accel{0};
    double omega{0};
};

struct Position2D {
    double x;
    double y;
};

struct TimeCostPair
{
    double planning_time;
    double cost;
};

class TimeUtil {
  public:
    static double NowSeconds() {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto duration = now.time_since_epoch();
        return duration_cast<microseconds>(duration).count() / 1e6;
    }

    static foxglove::schemas::Timestamp NowTimestamp() {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto duration = now.time_since_epoch();
        auto sec = duration_cast<seconds>(duration).count();
        auto nsec = duration_cast<nanoseconds>(duration).count() % 1000000000;
        foxglove::schemas::Timestamp ts;
        ts.sec = static_cast<uint32_t>(sec);
        ts.nsec = static_cast<uint32_t>(nsec);
        return ts;
    }

    static std::string NowTimeString() {
        using namespace std::chrono;
        auto tp = system_clock::now();
        auto t = system_clock::to_time_t(tp);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
        return oss.str();
    }
};

inline foxglove::schemas::Quaternion yaw_to_quaternion(double yaw) {
    foxglove::schemas::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw * 0.5);
    q.w = cos(yaw * 0.5);
    return q;
}

/**
 * @brief Template function to keep a fixed number of decimal places for floating-point numbers
 * @tparam digits Number of decimal places to keep, limited to 8 if exceeds
 * @param value Input floating-point number
 * @return double Floating-point number with specified decimal places
 */
template<uint8_t digits>
constexpr double to_fixed(double value) {
    constexpr int actual_digits = (digits > 8) ? 8 : digits;
    constexpr double multiplier = std::pow(10.0, actual_digits);
    return std::round(value * multiplier) / multiplier;
}
