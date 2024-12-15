/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 23:11:58
 * @LastEditTime: 2024-12-15 23:23:12
 * @FilePath: /dive-into-contingency-planning/include/utils.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <string>

struct Outlook;

namespace utils {

bool imread(std::string filename, Outlook& outlook);

} // namespace utils