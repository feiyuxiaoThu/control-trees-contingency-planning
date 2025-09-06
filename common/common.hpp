/**
 * @file common.hpp
 * @author puyu <yu.pu@qq.com>
 * @date 2025/8/3 00:17
 * Copyright (c) puyu. All rights reserved.
 */

#pragma once

struct State {
    double x{0};
    double y{0};
    double yaw{0};
    double velocity{0};
};

struct Control {
    double accel{0};
    double omega{0};
};
