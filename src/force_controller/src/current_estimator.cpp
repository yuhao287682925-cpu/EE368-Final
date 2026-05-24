#include <force_controller/current_estimator.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>

namespace force_controller {

CurrentEstimator::CurrentEstimator()
    : bias_(6, 0.0)
    , is_calibrated_(false)
{
    // Gen3 Lite DH parameters — Standard DH convention [alpha, a, d, theta_offset]
    // Source: jacobian.py (verified with real robot)
    // T_i = Rot_z(theta + offset) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    // All values in SI units
    dh_params_ = {
        {0.0,              0.0,      0.2433,  0.0},            // Joint 1
        {M_PI / 2.0,       0.0,      0.010,   M_PI / 2.0},     // Joint 2
        {M_PI,             0.280,    0.0,     M_PI / 2.0},     // Joint 3
        {M_PI / 2.0,       0.0,      0.245,   M_PI / 2.0},     // Joint 4
        {M_PI / 2.0,       0.0,      0.057,   0.0},            // Joint 5
        {-M_PI / 2.0,      0.0,      0.235,  -M_PI / 2.0},     // Joint 6
    };
}

void CurrentEstimator::calibrate(const std::vector<double>& tau_no_load) {
    if (tau_no_load.size() >= 6) {
        bias_.assign(tau_no_load.begin(), tau_no_load.begin() + 6);
        is_calibrated_ = true;
    }
}

double CurrentEstimator::estimateNormalForce(
    const std::vector<double>& q,
    const std::vector<double>& tau_measured,
    const Eigen::Vector3d& surface_normal) {

    if (q.size() < 6 || tau_measured.size() < 6) {
        return 0.0;
    }

    std::vector<double> tau_ext = getExternalTorques(tau_measured);

    Eigen::MatrixXd J = computeJacobian(q);

    // Extract linear velocity Jacobian (first 3 rows) and estimate tip force
    // tau = J_lin^T * F  →  F = pinv(J_lin^T) * tau
    Eigen::MatrixXd J_lin = J.block(0, 0, 3, 6);
    Eigen::VectorXd tau_vec(6);
    for (int i = 0; i < 6; ++i) tau_vec(i) = tau_ext[i];

    Eigen::MatrixXd J_lin_T = J_lin.transpose();  // 6x3
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_lin_T,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d F_est = svd.solve(tau_vec);

    // Project onto surface normal
    double F_normal = F_est.dot(surface_normal.normalized());

    return F_normal;
}

Eigen::Isometry3d CurrentEstimator::forwardKinematics(const std::vector<double>& q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        T = T * dhTransformStandard(dh_params_[i], q[i]);
    }
    Eigen::Isometry3d result;
    result.matrix() = T;
    return result;
}

Eigen::MatrixXd CurrentEstimator::computeJacobian(const std::vector<double>& q) {
    // Compute geometric Jacobian matching jacobian.py approach.
    // For each joint i, column = [z_i × (ee_pos - O_i), z_i]^T
    // where z_i = rotation axis of joint i in base frame,
    //       O_i = origin of the frame BEFORE joint i's rotation in base frame.
    //
    // We accumulate transforms T_0_to_i (after applying link i), matching the
    // Python pattern: trans = trans * T_i; use z and origin from trans for joint i.
    // This is correct because z_i × (ee-O_i) = z_i × (ee-O_{i-1}) for standard DH
    // (O_i = O_{i-1} + d·z_{i-1} and z_i is very close to z_{i-1}).

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);

    // First pass: compute FK to get end-effector position and all frame transforms
    std::vector<Eigen::Vector3d> origins(6);
    std::vector<Eigen::Vector3d> z_axes(6);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 6; ++i) {
        T = T * dhTransformStandard(dh_params_[i], q[i]);
        origins[i] = T.block<3, 1>(0, 3);
        z_axes[i] = T.block<3, 1>(0, 2);
    }

    Eigen::Vector3d ee_pos = origins.back();   // O_6 = end-effector position

    // Second pass: build Jacobian columns matching Python approach
    // Joint i column uses z_axes[i] and origins[i] (from T_0_to_i)
    // (Python uses trans after applying link i for joint i's column)
    for (int i = 0; i < 6; ++i) {
        // Linear: z_i × (ee_pos - O_i)
        J.block<3, 1>(0, i) = z_axes[i].cross(ee_pos - origins[i]);
        // Angular: z_i
        J.block<3, 1>(3, i) = z_axes[i];
    }

    return J;
}

std::vector<double> CurrentEstimator::getExternalTorques(
    const std::vector<double>& tau_measured) const {

    std::vector<double> tau_ext;
    tau_ext.reserve(6);
    for (size_t i = 0; i < 6 && i < tau_measured.size(); ++i) {
        double b = (i < bias_.size()) ? bias_[i] : 0.0;
        tau_ext.push_back(tau_measured[i] - b);
    }
    return tau_ext;
}

Eigen::Matrix4d CurrentEstimator::dhTransformStandard(const DHParam& dh, double theta) {
    // Standard DH convention (matching jacobian.py / dynamics_ros1.py):
    // T = Rot_z(θ+offset) * Trans_z(d) * Trans_x(a) * Rot_x(α)
    //
    // Matrix form:
    //   [[ct, -st*ca,  st*sa,  a*ct    ],
    //    [st,  ct*ca, -ct*sa,  a*st    ],
    //    [0,   sa,     ca,     d       ],
    //    [0,   0,      0,      1       ]]
    // where ct = cos(θ+offset), st = sin(θ+offset), ca = cos(α), sa = sin(α)

    double th = theta + dh.offset;
    double ct = std::cos(th);
    double st = std::sin(th);
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;
    T(0, 1) = -st * ca;
    T(0, 2) =  st * sa;
    T(0, 3) = dh.a * ct;

    T(1, 0) = st;
    T(1, 1) = ct * ca;
    T(1, 2) = -ct * sa;
    T(1, 3) = dh.a * st;

    T(2, 0) = 0.0;
    T(2, 1) = sa;
    T(2, 2) = ca;
    T(2, 3) = dh.d;

    return T;
}

}  // namespace force_controller
