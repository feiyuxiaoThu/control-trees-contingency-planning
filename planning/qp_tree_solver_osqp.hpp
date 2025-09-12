/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 15:56:05
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-13 14:58:47
 * @FilePath: /dive-into-contingency-planning/planning/qp_tree_solver_osqp.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved. 
 */

#pragma once

#include "planning/mpc_model.hpp"
#include "planning/qp_constraints.hpp"
#include "planning/qp_tree_solver_base.hpp"

class QPTreeSolverOSQP : public QP_tree_solver_base {
  public:
    QPTreeSolverOSQP(const MPCModel& mpc, double u_min, double u_max);

    Eigen::VectorXd solve(const Eigen::Vector2d& x0, const Eigen::Vector2d& xd,
                          const Constraints& k, int n_steps,
                          const std::vector<std::vector<int>>& varss,
                          const std::vector<std::vector<double>>& scaless);

    Eigen::MatrixXd get_H() const { return H; }
    Eigen::MatrixXd get_KA() const { return KA; }

  private:
    Eigen::VectorXd call_osqp_solver();

    // Eigen::VectorXd solve_unconstrained(const Eigen::Vector2d x0, const Eigen::Vector2d xd) {
    //     return -H.inverse() * F.transpose() * x0;
    // }

  protected:
    bool control_bounds_in_A_;

    Eigen::MatrixXd Q_bar;  // state cost
    Eigen::MatrixXd R_bar;  // control cost

    Eigen::MatrixXd S;
    Eigen::MatrixXd T;

    Eigen::MatrixXd F;
    Eigen::MatrixXd G;

    // final matrices used by the solver
    Eigen::MatrixXd H;
    Eigen::VectorXd C;
    Eigen::MatrixXd KA;
    Eigen::VectorXd Up;
    Eigen::VectorXd Lo;
};
