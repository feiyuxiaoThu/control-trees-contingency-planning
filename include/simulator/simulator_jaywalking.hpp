/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:25:00
 * @LastEditTime: 2024-12-15 22:56:51
 * @FilePath: /dive-into-contingency-planning/include/simulator/simulator_jaywalking.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include "common.hpp"
#include "simulator/simulator_base.hpp"

class SimulatorJaywalking : public SimulatorBase {
  private:
    Vehicle ego;
    Pedestrian pedestrian;

  public:
    SimulatorJaywalking(/* args */);
    ~SimulatorJaywalking() {}

    void refresh_scenario() override;
    void show_scenario() override;
};



