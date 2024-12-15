/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-14 00:46:19
 * @LastEditTime: 2024-12-16 00:37:49
 * @FilePath: /dive-into-contingency-planning/src/contingency_planning.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include "simulator/simulator_jaywalking.hpp"

int main(int argc, char** argv) {
    
    SimulatorJaywalking simulator;
    simulator.show_scenario();
    plt::show();

    return 0;
}
