#ifndef FORCE_CONTROLLER_CURRENT_ESTIMATOR_H
#define FORCE_CONTROLLER_CURRENT_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>

namespace force_controller {

/**
 * @brief Estimates end-effector contact force from joint torques.
 *
 * Uses the Jacobian transpose method:
 *   tau_external = J^T(q) * F_tip
 *   → F_tip ≈ pinv(J^T) * tau_external
 *
 * The Gen3 Lite has built-in joint torque sensors (accessible via
 * BaseCyclic_Feedback.actuators[].torque). We use a simplified
 * approach focusing on wrist joints where contact forces manifest.
 */
class CurrentEstimator {
public:
    CurrentEstimator();

    /**
     * @brief Estimate the normal contact force from joint torques.
     *
     * @param q             Joint angles [rad] (6 DOF for Gen3 Lite)
     * @param tau_measured  Measured joint torques [Nm]
     * @param surface_normal  Current surface normal vector in base frame
     * @return Estimated normal force [N] (positive = pushing into surface)
     */
    double estimateNormalForce(
        const std::vector<double>& q,
        const std::vector<double>& tau_measured,
        const Eigen::Vector3d& surface_normal);

    /**
     * @brief Calibrate torque bias (record zero-load torques).
     *        Should be called while the arm is not in contact.
     * @param tau_no_load  Measured joint torques with no external load
     */
    void calibrate(const std::vector<double>& tau_no_load);

    /**
     * @brief Compute the 6-DOF geometric Jacobian for the Gen3 Lite.
     * @param q  Joint angles [rad]
     * @return 6x6 Jacobian matrix [linear; angular]
     */
    Eigen::MatrixXd computeJacobian(const std::vector<double>& q);

    /**
     * @brief Get the estimated external joint torques (measured - bias).
     */
    std::vector<double> getExternalTorques(
        const std::vector<double>& tau_measured) const;

private:
    std::vector<double> bias_;   // Zero-load torque baseline (6 elements)
    bool is_calibrated_;

    // Gen3 Lite DH parameters (Standard DH convention, from jacobian.py)
    // [alpha, a, d, theta_offset], all in SI units
    struct DHParam {
        double alpha;   // Link twist [rad]
        double a;       // Link length [m]
        double d;       // Link offset [m]
        double offset;  // Joint angle offset [rad] (theta_actual = q + offset)
    };

    std::vector<DHParam> dh_params_;

    /**
     * @brief Compute single-link homogeneous transform (Standard DH).
     *        T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
     */
    Eigen::Matrix4d dhTransformStandard(const DHParam& dh, double theta);

    /**
     * @brief Full forward kinematics for the 6-DOF arm.
     * @param q Joint angles [rad] (6 elements)
     * @return T_base_to_ee homogeneous transform
     */
    Eigen::Isometry3d forwardKinematics(const std::vector<double>& q);
};

}  // namespace force_controller

#endif  // FORCE_CONTROLLER_CURRENT_ESTIMATOR_H
