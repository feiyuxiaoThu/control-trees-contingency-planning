#include "planning/qp_tree_solver_dec.hpp"

static arr convert(const Eigen::MatrixXd& M) {
    arr m(M.rows(), M.cols());

    for (uint i = 0; i < M.rows(); ++i) {
        for (uint j = 0; j < M.cols(); ++j) {
            m(i, j) = M(i, j);
        }
    }

    return m;
}

static Eigen::VectorXd convert(const arr& v) {
    Eigen::VectorXd V(v.d0);

    for (uint i = 0; i < V.rows(); ++i) {
        V(i) = v(i);
    }

    return V;
}

static arr createK(const Eigen::MatrixXd& KA) {
    arr K(2 * KA.rows(), KA.cols());

    for (uint i = 0; i < KA.rows(); ++i) {
        for (uint j = 0; j < KA.cols(); ++j) {
            K(i, j) = KA(i, j);
            K(i + KA.rows(), j) = -KA(i, j);
        }
    }

    return K;
}

static arr createU(const Eigen::VectorXd& Up, const Eigen::VectorXd& Lo) {
    arr u(2 * Up.rows());

    for (uint i = 0; i < Up.rows(); ++i) {
        u(i) = Up[i];
        u(i + Up.rows()) = -Lo[i];
    }

    return u;
}

static std::vector<int> to_local(const std::vector<int>& global_indices,
                                 const std::vector<int>& global_to_branch) {
    std::vector<int> local_indices(global_indices.size());

    for (uint j = 0; j < global_indices.size(); ++j) {
        local_indices[j] = global_to_branch[global_indices[j]];
    }

    return local_indices;
}

static std::vector<arr> get_compressed_masks(int n_steps, int dim,
                                             const std::vector<std::vector<int>>& varss,
                                             std::vector<int>& var,
                                             std::vector<int>& global_to_branch) {
    CHECK(!varss.empty() && var.empty(), "Preconditions not met");

    var.resize(varss.front().size());

    for (uint i = 0; i < var.size(); ++i) {
        var[i] = i;
    }

    std::vector<arr> masks(varss.size(), zeros(n_steps * dim));
    global_to_branch = std::vector<int>(n_steps);

    for (uint i = 0; i < varss.size(); ++i) {
        const auto& vars = varss[i];
        auto& mask = masks[i];

        for (uint j = 0; j < vars.size(); ++j) {
            for (uint k = 0; k < dim; ++k) {
                mask(dim * vars[j] + k) = 1.0;
                global_to_branch[vars[j]] = j;
            }
        }
    }

    return masks;
}

static std::vector<std::vector<double>> get_compressed_scales(
    const std::vector<std::vector<double>>& joint_scaless) {
    std::vector<std::vector<double>> scaless(joint_scaless.size());

    for (uint i = 0; i < joint_scaless.size(); ++i) {
        scaless[i] = std::vector<double>(joint_scaless[i].size(), joint_scaless[i].back());
    }

    return scaless;
}

static std::vector<double> get_one_scale(const std::vector<std::vector<double>>& joint_scaless) {
    return std::vector<double>(joint_scaless.front().size(), 1.0);
}

static arr get_belief_state(const std::vector<std::vector<double>>& joint_scaless) {
    arr bs = zeros(joint_scaless.size());

    for (uint i = 0; i < joint_scaless.size(); ++i) {
        bs(i) = joint_scaless[i].back();
    }

    return bs;
}

static std::unordered_map<int, Constraints> get_compressed_constraints(
    const Constraints& k, const std::vector<int>& var, const std::vector<int>& global_to_branch) {
    std::unordered_map<int, Constraints> constraints;
    constraints.reserve(k.xmaxs.size());

    for (uint i = 0; i < k.xmaxs.size(); ++i) {
        const auto& c = k.xmaxs[i];
        const auto& global_indices = std::get<3>(c);

        int branch = std::get<0>(c);

        // global to local
        const auto local_indices = to_local(global_indices, global_to_branch);

        auto cxmax = std::make_tuple(std::get<0>(c), std::get<1>(c), std::get<2>(c), local_indices);

        auto kit = constraints.find(branch);
        if (kit != constraints.end()) {
            kit->second.xmaxs.push_back(cxmax);
        } else {
            Constraints constraint(var.size(), std::vector<std::vector<int>>({var}));
            constraint.xmaxs = {cxmax};
            constraints.insert(std::make_pair(branch, constraint));
        }
    }

    return constraints;
}

QPTreeSolverDec::QPTreeSolverDec(const MPCModel& mpc, double u_min, double u_max)
    : QPTreeSolverBase(mpc, u_min, u_max), options(PARALLEL, true, NOOPT, false) {
    options.opt.verbose = 0;
    options.opt.aulaMuInc = 1;
}

Eigen::VectorXd QPTreeSolverDec::solve(const Eigen::Vector2d& x0, const Eigen::Vector2d& xd,
                                       const Constraints& joint_k, int n_steps,
                                       const std::vector<std::vector<int>>& joint_varss,
                                       const std::vector<std::vector<double>>& joint_scaless) {
    // generate compressed var and masks
    std::vector<int> var, global_to_branch;
    const auto masks =
        get_compressed_masks(n_steps, mpc.get_dim(), joint_varss, var, global_to_branch);
    const auto bs = get_belief_state(joint_scaless);
    //  const auto scales = get_one_scale(joint_scaless);
    const auto scaless = get_compressed_scales(joint_scaless);

    // compress constraints
    const auto ks = get_compressed_constraints(joint_k, var, global_to_branch);

    // build subproblems
    const auto branch_n_steps = var.size();
    std::vector<std::vector<int>> branch_varss({var});
    std::vector<std::shared_ptr<QP_Problem>> pbs;
    pbs.reserve(masks.size());

    std::vector<std::future<std::shared_ptr<QP_Problem>>> futures;

    futures.reserve(masks.size());
    for (uint i = 0; i < masks.size(); ++i) {
        futures.push_back(std::async(std::launch::async, [&, i]() {
            return build_qp(i, branch_n_steps, branch_varss, {scaless[i]} /*{scales}*/, ks, x0, xd);
        }));
    }

    for (auto& future : futures) {
        pbs.emplace_back(future.get());
    }

    // solve
    arr x = zeros(n_steps * mpc.get_dim());

    // DecOptConstrained<QP_Problem, BeliefState> opt(x, pbs, masks, BeliefState(bs), options);
    DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, masks, AverageUpdater(), options);

    opt.run();

    return convert(x);
}

std::shared_ptr<QP_Problem> QPTreeSolverDec::build_qp(
    int i, int n_steps, const std::vector<std::vector<int>>& varss,
    const std::vector<std::vector<double>>& scaless,
    const std::unordered_map<int, Constraints>& constraints, const Eigen::Vector2d& x0,
    const Eigen::Vector2d& xd) const {
    // build MPC matrices
    // costs (could be computed just once and scaled?)
    const auto S = mpc.get_S(n_steps, varss);
    const auto T = mpc.get_T(n_steps, varss);

    const auto Q_bar = mpc.get_Q_bar(n_steps, varss, scaless);
    const auto R_bar = mpc.get_R_bar(n_steps, varss, scaless);

    const auto H = 2 * (R_bar + S.transpose() * Q_bar * S);
    const auto F = 2 * (T.transpose() * Q_bar * S);
    const auto G = 2 * Q_bar * S;

    const Eigen::VectorXd& Xd = mpc.get_Xd(xd, n_steps);
    const auto C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();

    // constraints
    Eigen::MatrixXd KA;
    Eigen::VectorXd Up, Lo;

    const auto kit = constraints.find(i);
    if (kit != constraints.end()) {
        const auto& k = kit->second;
        const Eigen::VectorXd Xmax = k.getXmax();
        const Eigen::MatrixXd Sextract = k.getSextract();

        const auto nk = Sextract.rows() + H.rows();
        KA.resize(nk, H.rows());
        Up.resize(nk);
        Lo.resize(nk);

        // traj constraints
        KA.block(0, 0, Sextract.rows(), H.rows()) = Sextract * S;
        Up.head(Sextract.rows()) = Sextract * (Xmax - T * x0);
        Lo.head(Up.rows()) =
            Eigen::VectorXd::Constant(Up.rows(), std::numeric_limits<double>::lowest());

        // control bounds constraints
        KA.block(Sextract.rows(), 0, H.rows(), H.rows()) =
            Eigen::MatrixXd::Identity(H.rows(), H.rows());
        Up.tail(H.rows()) = Eigen::VectorXd::Constant(H.rows(), u_max_);
        Lo.tail(H.rows()) = Eigen::VectorXd::Constant(H.rows(), u_min_);
    } else  // no traj constraints, add control bounds only
    {
        // control bounds constraints
        KA = Eigen::MatrixXd::Identity(H.rows(), H.rows());
        Up = Eigen::VectorXd::Constant(H.rows(), u_max_);
        Lo = Eigen::VectorXd::Constant(H.rows(), u_min_);
    }

    // build QP matrices
    const auto P = convert(H);
    const auto q = convert(C);
    const auto K = createK(KA);
    const auto u = createU(Up, Lo);

    return std::make_shared<QP_Problem>(P, q, K, u);
}
