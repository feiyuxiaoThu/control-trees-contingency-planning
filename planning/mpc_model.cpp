/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 15:12:51
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 13:39:59
 * @FilePath: /dive-into-contingency-planning/planning/mpc_model.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/mpc_model.hpp"

MPCModel::MPCModel(double dt, double Q_v_weight, double R_u_weight) {
    // dynamic model
    A << 1, dt, 0, 1;
    B << 0, dt;

    // costs
    Q << 0, 0, 0, Q_v_weight;
    R << R_u_weight;
}

Eigen::VectorXd MPCModel::predict_trajectory(const Eigen::Vector2d& x0,
                                             const Eigen::VectorXd& U) const {
    auto n_steps = U.rows();

    Eigen::MatrixXd S = get_S(n_steps);
    Eigen::MatrixXd T = get_T(n_steps);

    return T * x0 + S * U;
}

Eigen::VectorXd MPCModel::predict_trajectory(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                                             const std::vector<std::vector<int>>& varss) const {
    auto n_steps = U.rows();

    Eigen::MatrixXd S = get_S(n_steps, varss);
    Eigen::MatrixXd T = get_T(n_steps, varss);

    return T * x0 + S * U;
}

double MPCModel::cost(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                      const Eigen::Vector2d& xd) const {
    auto n_steps = U.rows();

    Eigen::VectorXd Xd = get_Xd(xd, n_steps);

    Eigen::MatrixXd Q_bar = get_Q_bar(n_steps);
    Eigen::MatrixXd R_bar = get_R_bar(n_steps);

    const auto& X = predict_trajectory(x0, U);

    const auto cost = (X - Xd).transpose() * Q_bar * (X - Xd) + U.transpose() * R_bar * U;

    return cost(0, 0);
}

double MPCModel::cost(const Eigen::Vector2d& x, double u, const Eigen::Vector2d& xd) const {
    const auto _u = Eigen::VectorXd::Ones(1) * u;
    const auto& c = (x - xd).transpose() * Q * (x - xd) + _u.transpose() * R * _u;

    return c[0];
}

Eigen::VectorXd MPCModel::check_constraints(const Eigen::Vector2d& x0, const Eigen::VectorXd& U,
                                            const Eigen::Vector2d& xmax) const {
    auto n_steps = U.rows();

    auto X = predict_trajectory(x0, U);

    Eigen::VectorXd active = Eigen::VectorXd::Zero(X.rows());

    Eigen::VectorXd Xmax = Eigen::VectorXd::Zero(n_steps * 2);

    Eigen::VectorXd SU = get_S(n_steps) * U;
    Eigen::VectorXd Tx0 = get_T(n_steps) * x0;

    for (uint i = 0; i < n_steps; ++i) {
        Xmax(2 * i) = xmax(0);
        Xmax(2 * i + 1) = xmax(1);
    }

    for (auto i = 0; i < n_steps; ++i) {
        active[2 * i] = X[2 * i] <= Xmax[2 * i] ? 0 : 1.0;
        active[2 * i + 1] = X[2 * i + 1] <= Xmax[2 * i + 1] ? 0 : 1.0;

        assert((X[2 * i] <= Xmax[2 * i]) == (SU[2 * i] <= (Xmax - Tx0)[2 * i]));
        assert((X[2 * i + 1] <= Xmax[2 * i + 1]) == (SU[2 * i + 1] <= (Xmax - Tx0)[2 * i + 1]));
    }

    return active;
}

Eigen::VectorXd MPCModel::get_Xd(const Eigen::Vector2d& xd, int n_steps) const {
    Eigen::VectorXd Xd = Eigen::VectorXd(n_steps * 2);

    for (auto i = 0; i < n_steps; ++i) {
        Xd(2 * i) = xd(0);
        Xd(2 * i + 1) = xd(1);
    }
    return Xd;
}

Eigen::MatrixXd MPCModel::get_Q_bar(int n_steps) const {
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(2 * n_steps, 2 * n_steps);  // coef velocity
    for (auto i = 0; i < n_steps; ++i) {
        Q_bar.block(2 * i, 2 * i, 2, 2) = Q;
    }
    return Q_bar;
}

Eigen::MatrixXd MPCModel::get_Q_bar(int n_steps, const std::vector<std::vector<int>>& varss,
                                    const std::vector<std::vector<double>>& scaless) const {
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(2 * n_steps, 2 * n_steps);  // coef velocity
    assert(varss.size() == scaless.size());

    for (auto i = 0; i < varss.size(); ++i) {
        auto vars = varss[i];
        auto scales = scaless[i];

        for (auto j = 0; j < vars.size(); ++j) {
            auto p = scales[j];
            auto IJ = vars[j];
            Q_bar.block(2 * IJ, 2 * IJ, 2, 2) = p * Q;
        }
    }
    return Q_bar;
}

Eigen::MatrixXd MPCModel::get_R_bar(int n_steps) const {
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_steps, n_steps);
    for (auto i = 0; i < n_steps; ++i) {
        R_bar.block(i, i, 1, 1) = R;
    }
    return R_bar;
}

Eigen::MatrixXd MPCModel::get_R_bar(int n_steps, const std::vector<std::vector<int>>& varss,
                                    const std::vector<std::vector<double>>& scaless) const {
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_steps, n_steps);

    assert(varss.size() == scaless.size());

    for (auto i = 0; i < varss.size(); ++i) {
        auto vars = varss[i];
        auto scales = scaless[i];

        for (auto j = 0; j < vars.size(); ++j) {
            auto p = scales[j];
            auto IJ = vars[j];
            R_bar.block(IJ, IJ, 1, 1) = p * R;
        }
    }
    return R_bar;
}

Eigen::MatrixXd MPCModel::get_S(int n_steps) const {
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_steps * 2, n_steps);
    for (auto i = 0; i < n_steps; ++i) {
        for (auto j = 0; j <= i; ++j) {
            S.block(2 * i, j, 2, 1) = A.pow(i - j) * B;
        }
    }
    return S;
}

Eigen::MatrixXd MPCModel::get_S(int n_steps, const std::vector<std::vector<int>>& varss) const {
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_steps * 2, n_steps);

    for (auto i = 0; i < n_steps; i += 1) {
        S.block(2 * i, i, 2, 1) = B;
    }

    for (auto vars : varss) {
        for (auto i = 0; i < vars.size(); i += 1) {
            auto I = 2 * (vars[i]);  // where global on traj

            for (auto j = 0; j < i; j += 1) {
                auto J = vars[j];
                S.block(I, J, 2, 1) = A.pow(i - j) * B;
            }
        }
    }
    return S;
}

Eigen::MatrixXd MPCModel::get_T(int n_steps) const {
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_steps * 2, 2);
    for (auto i = 0; i < n_steps; i += 1) {
        T.block(2 * i, 0, 2, 2) = A.pow(i + 1);
    }
    return T;
}

Eigen::MatrixXd MPCModel::get_T(int n_steps, const std::vector<std::vector<int>>& varss) const {
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_steps * 2, 2);

    for (auto vars : varss) {
        for (auto i = 0; i < vars.size(); i += 1) {
            T.block(2 * (vars[i]), 0, 2, 2) = A.pow(i + 1);
        }
    }

    return T;
}
