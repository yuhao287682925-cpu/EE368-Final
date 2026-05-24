#include <robot_executor/pen_orientation.h>
#include <cmath>

namespace robot_executor {

Eigen::Quaterniond PenOrientation::computePenOrientation(
    const Eigen::Vector3d& surface_normal,
    const Eigen::Vector3d& travel_direction) {

    // Z-axis = -normal (pen points into surface)
    Eigen::Vector3d z_axis = -surface_normal.normalized();

    // X-axis = travel direction, projected onto plane perpendicular to Z
    Eigen::Vector3d x_axis = travel_direction - travel_direction.dot(z_axis) * z_axis;
    if (x_axis.norm() < 1e-10) {
        // Fallback: use global X or Y projected to surface
        x_axis = Eigen::Vector3d::UnitX();
        x_axis = x_axis - x_axis.dot(z_axis) * z_axis;
        if (x_axis.norm() < 1e-10) {
            x_axis = Eigen::Vector3d::UnitY();
            x_axis = x_axis - x_axis.dot(z_axis) * z_axis;
        }
    }
    x_axis.normalize();

    // Y-axis = Z × X (right-hand rule)
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    // Build rotation matrix
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    return Eigen::Quaterniond(R);
}

std::vector<Eigen::Quaterniond> PenOrientation::interpolateOrientation(
    const Eigen::Quaterniond& from,
    const Eigen::Quaterniond& to,
    int steps) {

    std::vector<Eigen::Quaterniond> result;
    result.reserve(steps + 2);
    result.push_back(from);

    if (steps <= 0) {
        result.push_back(to);
        return result;
    }

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / (steps + 1);
        result.push_back(from.slerp(t, to));
    }
    result.push_back(to);

    return result;
}

}  // namespace robot_executor
