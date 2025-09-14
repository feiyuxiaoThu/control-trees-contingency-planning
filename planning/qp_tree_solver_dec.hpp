/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-14 22:29:26
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 23:49:35
 * @FilePath: /dive-into-contingency-planning/planning/qp_tree_solver_dec.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "Core/array.h"
#include "OptimDec/qp_lagrangian.h"
#include "planning/mpc_model.hpp"
#include "planning/qp_constraints.hpp"
#include "planning/qp_tree_solver_base.hpp"

#include <unordered_map>

/*
 * Solve QP in decentralized fashion
 */
class QPTreeSolverDec : public QPTreeSolverBase {
  public:
    QPTreeSolverDec(const MPCModel& mpc, double u_min, double u_max);

    Eigen::VectorXd solve(const Eigen::Vector2d& x0, const Eigen::Vector2d& xd,
                          const Constraints& k, int n_steps,
                          const std::vector<std::vector<int>>& varss,
                          const std::vector<std::vector<double>>& scaless);

  private:
    /**
     * @brief build_qp
     * @param i branch id
     * @param n_steps number of steps on branch
     * @param varss local vars of the branch
     * @param scaless local scales of the branch
     * @param constraints mapping branch id -> constraints
     * @param x0 initial state
     * @param xd desired state
     * @return
     */
    std::shared_ptr<QP_Problem> build_qp(int i, int n_steps,
                                         const std::vector<std::vector<int>>& varss,
                                         const std::vector<std::vector<double>>& scaless,
                                         const std::unordered_map<int, Constraints>& constraints,
                                         const Eigen::Vector2d& x0,
                                         const Eigen::Vector2d& xd) const;

    DecOptConfig options;
};

// std::vector<arr> get_compressed_masks(int n_steps, int dim,
//                                       const std::vector<std::vector<int>>& varss,
//                                       std::vector<int>& var, std::vector<int>& global_to_branch);
// std::vector<std::vector<double>> get_compressed_scales(
//     const std::vector<std::vector<double>>& scaless);
// std::unordered_map<int, Constraints> get_compressed_constraints(
//     const Constraints& k, const std::vector<int>& var, const std::vector<int>& global_to_branch);
