#ifndef SOLVE_ANGLE__PLANNER_HPP_
#define SOLVE_ANGLE__PLANNER_HPP_

#include <Eigen/Dense>
#include <functional>
#include <memory>

namespace solve_angle {

constexpr double MPC_DT = 0.01;
constexpr int MPC_HORIZON = 100;
constexpr int MPC_HALF = MPC_HORIZON / 2;

using Trajectory = Eigen::Matrix<double, 2, MPC_HORIZON>;

struct Plan {
    bool valid;
    double yaw;
    double pitch;
    double yaw_vel;
    double pitch_vel;
    double yaw_acc;
    double pitch_acc;
    double fire_error;
};

class Planner {
public:
    Planner();
    ~Planner();

    void setup(double max_yaw_acc, double max_pitch_acc,
               double Q_yaw, double R_yaw,
               double Q_pitch, double R_pitch,
               double fire_thresh);

    Plan plan(double current_yaw, double current_yaw_vel,
              double current_pitch, double current_pitch_vel,
              const Trajectory& yaw_ref,
              const Trajectory& pitch_ref);

    void setEnabled(bool en) { enabled_ = en; }
    bool isEnabled() const { return enabled_; }

    // for debug: get MPC-solved full trajectory
    Trajectory last_yaw_traj() const { return last_yaw_traj_; }
    Trajectory last_pitch_traj() const { return last_pitch_traj_; }

private:
    bool enabled_ = true;
    double fire_thresh_ = 0.02;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    Trajectory last_yaw_traj_;
    Trajectory last_pitch_traj_;
};

}  // namespace solve_angle

#endif  // SOLVE_ANGLE__PLANNER_HPP_
