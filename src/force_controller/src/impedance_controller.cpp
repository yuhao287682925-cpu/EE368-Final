#include <force_controller/impedance_controller.h>
#include <algorithm>
#include <cmath>

namespace force_controller {

ImpedanceController::ImpedanceController()
    : Kp_(0.0003)
    , Kd_(0.00005)
    , M_(0.1)
    , F_desired_(1.0)
    , dz_max_(0.0005)
    , current_z_correction_(0.0)
    , prev_error_(0.0)
    , prev_dz_(0.0)
    , use_second_order_(false)
{
}

Eigen::Vector3d ImpedanceController::computePoseCorrection(
    double F_estimated,
    const Eigen::Vector3d& surface_normal,
    double dt) {

    double F_error = F_desired_ - F_estimated;

    double dz = 0.0;

    if (use_second_order_) {
        // Second-order virtual impedance:
        // M * ddz + D * dz_dot + K * dz = F_error
        // Discretized with forward Euler:
        // dz_dot = dz_dot_prev + dt * (F_error - Kp_ * z - Kd_ * dz_dot) / M
        double dz_accel = (F_error - Kp_ * current_z_correction_
                           - Kd_ * prev_dz_) / M_;
        double dz_dot = prev_dz_ + dt * dz_accel;
        dz = current_z_correction_ + dt * dz_dot;
        prev_dz_ = dz_dot;
    } else {
        // First-order (simple proportional):
        // dz = Kp * F_error
        dz = Kp_ * F_error;
    }

    // Clamp
    dz = std::max(-dz_max_, std::min(dz_max_, dz));

    current_z_correction_ += dz;
    prev_error_ = F_error;

    return surface_normal.normalized() * current_z_correction_;
}

void ImpedanceController::reset() {
    current_z_correction_ = 0.0;
    prev_error_ = 0.0;
    prev_dz_ = 0.0;
}

void ImpedanceController::setParams(double Kp, double Kd,
                                     double F_desired, double dz_max) {
    Kp_ = Kp;
    Kd_ = Kd;
    F_desired_ = F_desired;
    dz_max_ = dz_max;
}

}  // namespace force_controller
