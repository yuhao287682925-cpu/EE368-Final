#ifndef ROBOT_EXECUTOR_PEN_ORIENTATION_H
#define ROBOT_EXECUTOR_PEN_ORIENTATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace robot_executor {

/**
 * @brief Computes and interpolates pen-tip orientations.
 *
 * Pen frame convention:
 *   Z-axis = -surface_normal  (pen points into the block surface)
 *   X-axis = travel_direction (along the drawing path)
 *   Y-axis = Z x X           (right-hand rule)
 */
class PenOrientation {
public:
    PenOrientation() = default;

    /**
     * @brief Compute pen orientation quaternion.
     * @param surface_normal   Unit vector normal to the surface (pointing outward)
     * @param travel_direction Unit vector along drawing direction
     * @return Quaternion representing pen tip orientation
     */
    Eigen::Quaterniond computePenOrientation(
        const Eigen::Vector3d& surface_normal,
        const Eigen::Vector3d& travel_direction);

    /**
     * @brief SLERP interpolation between two orientations.
     * @param from   Start orientation
     * @param to     End orientation
     * @param steps  Number of interpolation steps
     * @return Interpolated orientations (including from and to)
     */
    std::vector<Eigen::Quaterniond> interpolateOrientation(
        const Eigen::Quaterniond& from,
        const Eigen::Quaterniond& to,
        int steps);
};

}  // namespace robot_executor

#endif  // ROBOT_EXECUTOR_PEN_ORIENTATION_H
