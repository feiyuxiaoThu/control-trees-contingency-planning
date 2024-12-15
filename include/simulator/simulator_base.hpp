/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:24:08
 * @LastEditTime: 2024-12-16 00:34:07
 * @FilePath: /dive-into-contingency-planning/include/simulator/simulator_base.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <chrono>
#include <thread>

#include "common.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class SimulatorBase {
  private:
    /* data */
  public:
    SimulatorBase(/* args */) {}
    virtual ~SimulatorBase() {}

    virtual void refresh_scenario() = 0;
    virtual void show_scenario() = 0;
    void check10hz(std::chrono::steady_clock::time_point tbegin);
    void imshow(const Outlook& img, const std::vector<double>& state);
};

