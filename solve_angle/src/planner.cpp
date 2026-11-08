#include "solve_angle/planner.hpp"

#include <cmath>

// TinyMPC API
#include "tinympc/tiny_api.hpp"

namespace solve_angle {

struct Planner::Impl {
    TinySolver* yaw_solver = nullptr;
    TinySolver* pitch_solver = nullptr;
};

Planner::Planner()
    : impl_(std::make_unique<Impl>()) {}

Planner::~Planner() {
    // TinyMPC solvers use malloc internally; no explicit free needed in API.
    // For safety, we just null them.
    impl_->yaw_solver = nullptr;
    impl_->pitch_solver = nullptr;
}

void Planner::setup(double max_yaw_acc, double max_pitch_acc,
                    double Q_yaw, double R_yaw,
                    double Q_pitch, double R_pitch,
                    double fire_thresh)
{
    fire_thresh_ = fire_thresh;
    const int nx = 2, nu = 1, N = MPC_HORIZON;

    // System model: x_{k+1} = A x_k + B u_k
    // state = [pos, vel], input = acc
    Eigen::MatrixXd A{{1, MPC_DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {MPC_DT}};
    Eigen::VectorXd f{{0, 0}};

    // --- Yaw solver ---
    Eigen::Vector2d Qy(Q_yaw, Q_yaw);
    Eigen::Matrix<double, 1, 1> Ry; Ry(0,0) = R_yaw;
    tiny_setup(&impl_->yaw_solver, A, B, f, Qy.asDiagonal(), Ry.asDiagonal(),
               1.0, nx, nu, N, 0);

    {
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N,  1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N-1, -max_yaw_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N-1,  max_yaw_acc);
        tiny_set_bound_constraints(impl_->yaw_solver, x_min, x_max, u_min, u_max);
    }
    impl_->yaw_solver->settings->max_iter = 10;

    // --- Pitch solver ---
    Eigen::Vector2d Qp(Q_pitch, Q_pitch);
    Eigen::Matrix<double, 1, 1> Rp; Rp(0,0) = R_pitch;
    tiny_setup(&impl_->pitch_solver, A, B, f, Qp.asDiagonal(), Rp.asDiagonal(),
               1.0, nx, nu, N, 0);

    {
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(nx, N, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(nx, N,  1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(nu, N-1, -max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(nu, N-1,  max_pitch_acc);
        tiny_set_bound_constraints(impl_->pitch_solver, x_min, x_max, u_min, u_max);
    }
    impl_->pitch_solver->settings->max_iter = 10;
}

Plan Planner::plan(double current_yaw, double current_yaw_vel,
                   double current_pitch, double current_pitch_vel,
                   const Trajectory& yaw_ref,
                   const Trajectory& pitch_ref)
{
    // When disabled, bypass MPC and return raw values
    if (!enabled_) {
        Plan p;
        p.valid      = true;
        p.yaw        = current_yaw;
        p.pitch      = current_pitch;
        p.yaw_vel    = current_yaw_vel;
        p.pitch_vel  = current_pitch_vel;
        p.yaw_acc    = 0.0;
        p.pitch_acc  = 0.0;
        p.fire_error = 0.0;
        return p;
    }

    Plan p;
    p.valid = false;

    // --- Yaw MPC ---
    {
        Eigen::VectorXd x0(2);
        x0 << current_yaw, current_yaw_vel;
        tiny_set_x0(impl_->yaw_solver, x0);
        impl_->yaw_solver->work->Xref = yaw_ref;
        int status = tiny_solve(impl_->yaw_solver);
        if (status != 0) return p;
    }

    // --- Pitch MPC ---
    {
        Eigen::VectorXd x0(2);
        x0 << current_pitch, current_pitch_vel;
        tiny_set_x0(impl_->pitch_solver, x0);
        impl_->pitch_solver->work->Xref = pitch_ref;
        int status = tiny_solve(impl_->pitch_solver);
        if (status != 0) return p;
    }

    // Extract result at HALF_HORIZON (t = 0.5s)
    p.valid      = true;
    p.yaw        = impl_->yaw_solver->work->x(0, MPC_HALF);
    p.yaw_vel    = impl_->yaw_solver->work->x(1, MPC_HALF);
    p.yaw_acc    = impl_->yaw_solver->work->u(0, MPC_HALF);
    p.pitch      = impl_->pitch_solver->work->x(0, MPC_HALF);
    p.pitch_vel  = impl_->pitch_solver->work->x(1, MPC_HALF);
    p.pitch_acc  = impl_->pitch_solver->work->u(0, MPC_HALF);

    // Fire error: L2 difference between ref and solution at t=0.52s (offset 2)
    int check_idx = std::min(MPC_HALF + 2, MPC_HORIZON - 1);
    double dy = yaw_ref(0, check_idx)   - impl_->yaw_solver->work->x(0, check_idx);
    double dp = pitch_ref(0, check_idx) - impl_->pitch_solver->work->x(0, check_idx);
    p.fire_error = std::sqrt(dy*dy + dp*dp);

    // Save for debug
    last_yaw_traj_   = impl_->yaw_solver->work->x.topRows<2>();
    last_pitch_traj_ = impl_->pitch_solver->work->x.topRows<2>();

    return p;
}

}  // namespace solve_angle
