/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 16:00:17
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 23:55:29
 * @FilePath: /dive-into-contingency-planning/planning/stopline_qp_tree.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/control_tree.hpp"
#include "common/protos/planning_info.pb.h"
#include "planning/mpc_model.hpp"
#include "planning/qp_tree_solver_osqp.hpp"
#include "planning/qp_tree_solver_dec.hpp"
#include "simulator/pedestrian.hpp"

#include <yaml-cpp/yaml.h>

struct Stopline {
    double x;
    double p;
};

class StopLineQPTree {
  public:
    StopLineQPTree() = delete;
    StopLineQPTree(const YAML::Node& config);
    StopLineQPTree(int n_branches, int steps_per_phase);

    void set_desired_speed(double desired_speed) { v_desired_ = desired_speed; }

    void update_stopline(const State &ego_current_state,
                         std::vector<std::shared_ptr<const Pedestrian>> pedestrians);

    TimeCostPair plan(const State& current_state,
                      std::vector<std::shared_ptr<const Pedestrian>> pedestrians);

    bool validate_and_save_solution(const Eigen::VectorXd &U, const State &o);

    planning::protos::PlanningInfo get_debug_result(const State& current_state);

    Control get_control() const { return Control{U_sol_.size() > 0 ? U_sol_[0] : 0.0, 0.0}; }

  private:
    void create_tree();
    bool valid(const Eigen::VectorXd& U, const Eigen::VectorXd& X) const;
    void init_logger(const std::string& log_level_str = "info");

    uint32_t plan_seq_{0};
    double solve_cost_time_ms_{0.0};
    double solution_cost_{0.0};
    std::shared_ptr<spdlog::logger> logger_ = nullptr;

    // params
    const int n_branches_;
    const uint steps_;
    double u_min_{-6.0};
    double u_max_{2.0};

    // target: params than can be adapted
    double v_desired_;
    std::vector<Stopline> stoplines_;

    // tree
    std::shared_ptr<TreePb> tree_;
    MPCModel model_;
    std::unique_ptr<QPTreeSolverBase> solver_{nullptr};

    // results
    Eigen::Vector2d x0_;
    Eigen::VectorXd U_sol_;
    Eigen::VectorXd X_sol_;

    // state;
    bool optimization_run_;
    bool optimization_error_;
};

Eigen::VectorXd emergency_brake(const double v, const TreePb &, int steps_per_phase, double u);
std::vector<double> fuse_probabilities(const std::vector<Stopline> &, int n);  // n = n_branches -1
Eigen::VectorXd emergency_brake(double v, int n_phases, int steps_per_phase, double u);
