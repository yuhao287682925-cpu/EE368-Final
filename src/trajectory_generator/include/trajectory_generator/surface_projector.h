#ifndef TRAJECTORY_GENERATOR_SURFACE_PROJECTOR_H
#define TRAJECTORY_GENERATOR_SURFACE_PROJECTOR_H

#include <trajectory_generator/types.h>
#include <block_drawing_msgs/SurfaceTrajectory.h>
#include <Eigen/Geometry>
#include <vector>

namespace trajectory_generator {

/**
 * @brief Core projection: maps 2D pattern polylines onto 3D block faces.
 *
 * Given the block pose (T_block_base) and dimensions (L, W, H),
 * projects 2D points onto the specified face, computing both
 * position (flange-compensated) and orientation (pen normal to surface).
 */
class SurfaceProjector {
public:
    SurfaceProjector();

    /**
     * @brief Set the block pose in the robot base frame.
     * @param T_block_base  Homogeneous transform: block center → base frame
     */
    void setBlockPose(const Eigen::Isometry3d& T_block_base);

    /**
     * @brief Set the block dimensions.
     * @param L  Length (X-direction)
     * @param W  Width  (Y-direction)
     * @param H  Height (Z-direction)
     */
    void setBlockSize(double L, double W, double H);

    /**
     * @brief Set per-face projection offsets (mm, in face u/v coordinates).
     *        Pattern (0,0) maps to face_origin + u_offset*u_axis + v_offset*v_axis.
     * @param face_id  Face index [0..4]
     * @param u_offset_mm  Offset along face u-axis [mm]
     * @param v_offset_mm  Offset along face v-axis [mm]
     */
    void setFaceOffset(int face_id, double u_offset_mm, double v_offset_mm);

    /**
     * @brief Project a 2D polyline onto a specific block face (isolated, independent per face).
     */
    block_drawing_msgs::SurfaceTrajectory project2DToFace(
        const Polyline& polyline,
        int face_id,
        double pen_tip_offset);

    /**
     * @brief Continuous cross-face projection.
     *
     * Pattern center is anchored at (center_face, center_u_mm, center_v_mm).
     * Each 2D point (x,y) maps to (center_u + x, center_v + y) starting from
     * center_face. If (u,v) exceeds face bounds, the trajectory crosses to the
     * adjacent face without lifting the pen.
     *
     * @param polyline        2D polyline in mm (pattern coordinates)
     * @param center_face     Face where pattern center is located [0..4]
     * @param center_u_mm     Pattern center u-coordinate on center_face [mm]
     * @param center_v_mm     Pattern center v-coordinate on center_face [mm]
     * @param pen_tip_offset  Flange-to-pen-tip distance in meters
     * @return Single SurfaceTrajectory spanning all touched faces
     */
    block_drawing_msgs::SurfaceTrajectory project2DContinuous(
        const Polyline& polyline,
        int center_face,
        double center_u_mm,
        double center_v_mm,
        double pen_tip_offset);

    /**
     * @brief Get the base-frame face frame for a given face.
     * @param face_id  Face index [0..4]
     * @return FaceFrame with origin, u/v axes, normal in base frame
     */
    FaceFrame getFaceFrame(int face_id) const;

    /**
     * @brief Generate lift-off and approach waypoints for face transitions.
     *
     * @param from_face      Source face index
     * @param to_face        Target face index
     * @param last_point_2d  Last 2D point on source face (mm)
     * @param first_point_2d First 2D point on target face (mm)
     * @param lift_height    Lift distance along face normal (m)
     * @param pen_tip_offset Flange-to-pen-tip distance (m)
     * @return vector of waypoints: [lifted_from, lifted_to, approach_to]
     */
    std::vector<geometry_msgs::Pose> generateTransitionWaypoints(
        int from_face, int to_face,
        const Point2D& last_point_2d,
        const Point2D& first_point_2d,
        double lift_height,
        double pen_tip_offset);

private:
    /**
     * @brief Compute face frames in block-local coordinates,
     *        then transform to base frame via T_block_base_.
     */
    void updateFaceFrames();

    /**
     * @brief Compute pen orientation given travel direction and surface normal.
     * Z-axis = -normal (into surface), X-axis = travel direction.
     */
    Eigen::Quaterniond computePenOrientation(
        const Eigen::Vector3d& travel_direction,
        const Eigen::Vector3d& surface_normal) const;

    /**
     * @brief Convert Eigen pose to ROS geometry_msgs/Pose.
     */
    geometry_msgs::Pose eigenToRosPose(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation) const;

    /**
     * @brief Resolve a 2D point on the block surface to a specific face.
     *        Handles boundary crossing via face adjacency lookup.
     * @param[in,out] face_id  Starting face, updated to final face
     * @param[in,out] u_mm     u-coordinate [mm], updated for new face
     * @param[in,out] v_mm     v-coordinate [mm], updated for new face
     */
    void resolveFacePoint(int& face_id, double& u_mm, double& v_mm) const;

    /**
     * @brief Get face bounds in mm (u_min, u_max, v_min, v_max).
     */
    void getFaceBounds(int face_id, double& u_min, double& u_max,
                       double& v_min, double& v_max) const;

    Eigen::Isometry3d T_block_base_;
    double L_, W_, H_;
    double face_offsets_u_[5];  // mm, per-face u-axis offset
    double face_offsets_v_[5];  // mm, per-face v-axis offset
    FaceFrame face_frames_[5];  // 0..4: top, front, right, back, left
    bool block_pose_set_;
    bool block_size_set_;
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR_SURFACE_PROJECTOR_H
