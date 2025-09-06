/**
 * @file main.cpp
 * @author puyu <yu.pu@qq.com>
 * @date 2025/8/3 00:23
 * Copyright (c) puyu. All rights reserved.
 */

#include "simulator/simulator.hpp"

int main() {
    Simulator simulator("config.yaml");
    simulator.start();

    while (true) {
        // nothing
    }

    return 0;
}
