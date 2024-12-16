/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:22:05
 * @LastEditTime: 2024-12-17 00:30:15
 * @FilePath: /dive-into-contingency-planning/include/common.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <vector>
#include <chrono>
#include <filesystem>

#include <spdlog/spdlog.h>

struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double a = 0.0;
    double da = 0.0;
    double relative_time = 0.0;

    std::vector<double> to_vector() {
        return {x, y, theta, v, a, da, relative_time};
    }
};

struct Trajectory {
    std::vector<TrajectoryPoint> trajectory;
    double probability;
    uint16_t branch_index;
};

struct Outlook {
    int rows;
    int cols;
    int channels;
    std::vector<float> data;
    double visual_width;
    double visual_height;
};

enum class Intention {RIGHT, LEFT};

struct Agent {
    TrajectoryPoint cur_state;
    Outlook outlook;
    Intention intent;
};

struct Vehicle : public Agent {
    Vehicle();

    std::vector<Trajectory> trajectories;
    double target_velocity;
};

struct Pedestrian : public Agent {
    Pedestrian();
};

class TicToc {
  public:
    TicToc(void) { tic(); }

    void tic(void) { start = std::chrono::system_clock::now(); }

    double toc(void) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count();
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
};
