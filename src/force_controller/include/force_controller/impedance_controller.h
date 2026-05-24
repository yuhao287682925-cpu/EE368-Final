#ifndef FORCE_CONTROLLER_IMPEDANCE_CONTROLLER_H
#define FORCE_CONTROLLER_IMPEDANCE_CONTROLLER_H

#include <Eigen/Dense>

namespace force_controller {

/**
 * @brief Virtual impedance controller for pen-tip force regulation.
 *
 * Two modes:
 *
 * 1. First-order (simple):
 *    Δz = Kp * (F_desired - F_estimated)
 *    clamp(Δz, -dz_max, +dz_max)
 *
 * 2. Second-order (smoother):
 *    M * Δz̈ + D * Δż + K * Δz = F_error
 *    Discretized with forward Euler.
 */
class ImpedanceController {
public:
    ImpedanceController();

    /**
     * @brief Compute pose correction from force error.
     *
     * @param F_estimated   Current estimated normal force [N]
     * @param surface_normal Current surface normal (outward) in base frame
     * @param dt            Control period [s]
     * @return Position correction vector in base frame [m] (along surface normal)
     */
    Eigen::Vector3d computePoseCorrection(
        double F_estimated,
        const Eigen::Vector3d& surface_normal,
        double dt);

    /**
     * @brief Reset internal state (call at start of each face).
     */
    void reset();

    /**
     * @brief Set controller parameters.
     */
    void setParams(double Kp, double Kd, double F_desired, double dz_max);

    /**
     * @brief Get the current accumulated correction.
     */
    double getCurrentCorrection() const { return current_z_correction_; }

    /**
     * @brief Use second-order impedance model (true) or first-order (false).
     */
    void setUseSecondOrder(bool use) { use_second_order_ = use; }

private:
    double Kp_;                    // Proportional gain [m/N]
    double Kd_;                    // Derivative gain [m.s/N]
    double M_;                     // Virtual mass [kg] for 2nd order
    double F_desired_;             // Desired normal force [N]
    double dz_max_;                // Max per-step correction [m]
    double current_z_correction_;  // Accumulated Z correction [m]
    double prev_error_;            // Previous force error [N]
    double prev_dz_;               // Previous correction velocity [m/s]
    bool use_second_order_;
};

}  // namespace force_controller

#endif  // FORCE_CONTROLLER_IMPEDANCE_CONTROLLER_H
