/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 15:12:59
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 15:44:20
 * @FilePath: /dive-into-contingency-planning/planning/mpc_model.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

class MPCModel {
  public:
    MPCModel(double dt, double Q_v_weight, double R_u_weight);

    Eigen::VectorXd predict_trajectory(const Eigen::Vector2d& x0, const Eigen::VectorXd& U) const;
    Eigen::VectorXd predict_trajectory(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                                       const std::vector<std::vector<int>>& varss) const;

    double cost(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                const Eigen::Vector2d& xd) const;  // cost of full control sequence
    double cost(const Eigen::Vector2d& x, double u,
                const Eigen::Vector2d& xd) const;  // cost of single step

    Eigen::VectorXd check_constraints(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                                      const Eigen::Vector2d& xmax) const;

    Eigen::VectorXd get_Xd(const Eigen::Vector2d& xd, int n_steps) const;

    Eigen::MatrixXd get_Q_bar(int n_steps) const;
    Eigen::MatrixXd get_Q_bar(int n_steps, const std::vector<std::vector<int>>& varss,
                              const std::vector<std::vector<double>>& scaless) const;

    Eigen::MatrixXd get_R_bar(int n_steps) const;
    Eigen::MatrixXd get_R_bar(int n_steps, const std::vector<std::vector<int>>& varss,
                              const std::vector<std::vector<double>>& scaless) const;

    Eigen::MatrixXd get_S(int n_steps) const;
    Eigen::MatrixXd get_S(int n_steps, const std::vector<std::vector<int>>& varss) const;

    Eigen::MatrixXd get_T(int n_steps) const;
    Eigen::MatrixXd get_T(int n_steps, const std::vector<std::vector<int>>& varss) const;

    int get_dim() const { return 1; }

  private:
    Eigen::Matrix2d A;  // system model
    Eigen::Vector2d B;  // system model

    Eigen::Matrix2d Q;              // state costs
    Eigen::Matrix<double, 1, 1> R;  // control costs
};
