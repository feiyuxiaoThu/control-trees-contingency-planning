/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 15:52:19
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-13 14:59:02
 * @FilePath: /dive-into-contingency-planning/planning/qp_tree_solver_base.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "planning/mpc_model.hpp"
#include "planning/qp_constraints.hpp"

class QP_tree_solver_base {
  public:
    QP_tree_solver_base(const MPCModel& mpc, double u_min, double u_max)
        : mpc(mpc), u_min_(u_min), u_max_(u_max) {}

    virtual Eigen::VectorXd solve(const Eigen::Vector2d& x0, const Eigen::Vector2d& xd,
                                  const Constraints& k, int n_steps,
                                  const std::vector<std::vector<int>>& varss,
                                  const std::vector<std::vector<double>>& scaless) = 0;

  protected:
    const MPCModel& mpc;

    const double u_min_;
    const double u_max_;
};
