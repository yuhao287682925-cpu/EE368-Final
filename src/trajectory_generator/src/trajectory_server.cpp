#include <ros/ros.h>
#include <block_drawing_msgs/GenerateTrajectory.h>
#include <block_drawing_msgs/SetBlockPose.h>
#include <trajectory_generator/svg_reader.h>
#include <trajectory_generator/surface_projector.h>
#include <trajectory_generator/types.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

using namespace trajectory_generator;

class TrajectoryServer {
public:
    TrajectoryServer(ros::NodeHandle& nh)
        : nh_(nh)
        , block_pose_set_(false)
    {
        // Services
        srv_set_block_ = nh_.advertiseService(
            "/set_block_pose", &TrajectoryServer::handleSetBlockPose, this);
        srv_generate_ = nh_.advertiseService(
            "/generate_trajectory", &TrajectoryServer::handleGenerateTrajectory, this);

        // Visualization
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/trajectory_markers", 10, true);

        // Parameters
        nh_.param("pen_tip_offset", pen_tip_offset_, 0.12);
        nh_.param("lift_height", lift_height_, 0.05);
        nh_.param("default_block_L", default_L_, 0.2);
        nh_.param("default_block_W", default_W_, 0.2);
        nh_.param("default_block_H", default_H_, 0.15);

        // Set default block size
        projector_.setBlockSize(default_L_, default_W_, default_H_);

        ROS_INFO("Trajectory server ready.");
    }

private:
    bool handleSetBlockPose(block_drawing_msgs::SetBlockPose::Request& req,
                            block_drawing_msgs::SetBlockPose::Response& res) {
        try {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.translation() = Eigen::Vector3d(
                req.block_pose.position.x,
                req.block_pose.position.y,
                req.block_pose.position.z);
            Eigen::Quaterniond q(
                req.block_pose.orientation.w,
                req.block_pose.orientation.x,
                req.block_pose.orientation.y,
                req.block_pose.orientation.z);
            T.linear() = q.toRotationMatrix();

            projector_.setBlockPose(T);
            projector_.setBlockSize(req.L, req.W, req.H);

            // Apply per-face projection offsets
            for (int i = 0; i < 5; ++i) {
                double u_off = (i < static_cast<int>(req.face_offset_u.size())) ? req.face_offset_u[i] : 0.0;
                double v_off = (i < static_cast<int>(req.face_offset_v.size())) ? req.face_offset_v[i] : 0.0;
                projector_.setFaceOffset(i, u_off, v_off);
            }

            // Store continuous mode params
            use_continuous_ = (req.center_face >= 0 && req.center_face <= 4);
            center_face_ = req.center_face;
            center_u_mm_ = req.center_u_mm;
            center_v_mm_ = req.center_v_mm;

            block_pose_set_ = true;

            res.success = true;
            std::string mode = use_continuous_ ? "continuous" : "per-face";
            res.message = "Block pose, size and offsets set (" + mode + " mode).";
            ROS_INFO("Block pose set: pos=(%.3f, %.3f, %.3f), mode=%s",
                     req.block_pose.position.x, req.block_pose.position.y,
                     req.block_pose.position.z, mode.c_str());
        } catch (const std::exception& e) {
            res.success = false;
            res.message = std::string("Error: ") + e.what();
            ROS_ERROR("%s", res.message.c_str());
        }
        return true;
    }

    bool handleGenerateTrajectory(block_drawing_msgs::GenerateTrajectory::Request& req,
                                  block_drawing_msgs::GenerateTrajectory::Response& res) {
        if (!block_pose_set_) {
            res.success = false;
            res.message = "Block pose not set. Call /set_block_pose first.";
            return true;
        }

        try {
            std::vector<Polyline> polylines;

            if (!req.svg_file.empty()) {
                polylines = reader_.loadFromFile(
                    req.svg_file, req.target_width_mm, req.target_height_mm);
                ROS_INFO("Loaded %zu polylines from SVG: %s",
                         polylines.size(), req.svg_file.c_str());
            } else {
                // Use built-in test pattern
                if (req.test_pattern == "circle") {
                    double r = std::min(req.target_width_mm, req.target_height_mm) / 2.0;
                    polylines = reader_.generateTestCircle(r);
                    ROS_INFO("Using built-in circle pattern (r=%.1f mm).", r);
                } else {
                    polylines = reader_.generateTestSquare(
                        req.target_width_mm, req.target_height_mm);
                    ROS_INFO("Using built-in square pattern.");
                }
            }

            if (use_continuous_) {
                // Continuous cross-face mode: one trajectory spanning faces
                for (const auto& pl : polylines) {
                    auto traj = projector_.project2DContinuous(
                        pl, center_face_, center_u_mm_, center_v_mm_, pen_tip_offset_);
                    res.trajectories.push_back(traj);
                    ROS_INFO("Continuous: %zu waypoints, total arc %.3f m",
                             traj.waypoints.size(),
                             traj.arc_lengths.empty() ? 0.0 : traj.arc_lengths.back());
                }
            } else {
                // Per-face mode: each face gets its own copy
                for (int face_id : req.faces) {
                    if (face_id < 0 || face_id > 4) {
                        ROS_WARN("Invalid face_id %d, skipping.", face_id);
                        continue;
                    }
                    for (const auto& pl : polylines) {
                        auto traj = projector_.project2DToFace(pl, face_id, pen_tip_offset_);
                        res.trajectories.push_back(traj);
                        ROS_INFO("Face %d: %zu waypoints, total arc %.3f m",
                                 face_id, traj.waypoints.size(),
                                 traj.arc_lengths.empty() ? 0.0 : traj.arc_lengths.back());
                    }
                }
            }

            // Publish visualization markers
            publishMarkers(res.trajectories);

            res.success = true;
            res.message = "Generated " + std::to_string(res.trajectories.size())
                        + " trajectory segments.";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = std::string("Error: ") + e.what();
            ROS_ERROR("%s", res.message.c_str());
        }
        return true;
    }

    void publishMarkers(
        const std::vector<block_drawing_msgs::SurfaceTrajectory>& trajectories) {

        visualization_msgs::MarkerArray markers;
        int marker_id = 0;

        for (const auto& traj : trajectories) {
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "world";
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "trajectory";
            line_marker.id = marker_id++;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.002;  // 2mm line width

            // Color by face
            switch (traj.face_id) {
                case 0: line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 0.0; break; // Red: top
                case 1: line_marker.color.r = 0.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0; break; // Green: front
                case 2: line_marker.color.r = 0.0; line_marker.color.g = 0.0; line_marker.color.b = 1.0; break; // Blue: right
                case 3: line_marker.color.r = 1.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0; break; // Yellow: back
                case 4: line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 1.0; break; // Magenta: left
            }
            line_marker.color.a = 1.0;

            for (const auto& wp : traj.waypoints) {
                // Show actual flange positions (surface position by adding back tip offset along z)
                // For visualization we just show the flange waypoints
                geometry_msgs::Point p;
                p.x = wp.position.x;
                p.y = wp.position.y;
                p.z = wp.position.z;
                line_marker.points.push_back(p);
            }
            markers.markers.push_back(line_marker);

            // Add arrow markers for orientation at every 10th waypoint
            int step = std::max(1, static_cast<int>(traj.waypoints.size()) / 20);
            for (size_t i = 0; i < traj.waypoints.size(); i += step) {
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = "world";
                arrow.header.stamp = ros::Time::now();
                arrow.ns = "orientation";
                arrow.id = marker_id++;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.action = visualization_msgs::Marker::ADD;
                arrow.scale.x = 0.01;  // shaft diameter
                arrow.scale.y = 0.015; // head diameter
                arrow.scale.z = 0.0;   // not used

                arrow.pose = traj.waypoints[i];
                arrow.color.r = 0.0;
                arrow.color.g = 1.0;
                arrow.color.b = 1.0;
                arrow.color.a = 0.8;

                markers.markers.push_back(arrow);
            }
        }

        pub_markers_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::ServiceServer srv_set_block_;
    ros::ServiceServer srv_generate_;
    ros::Publisher pub_markers_;

    SvgReader reader_;
    SurfaceProjector projector_;
    bool block_pose_set_;
    bool use_continuous_;
    int center_face_;
    double center_u_mm_, center_v_mm_;

    double pen_tip_offset_;
    double lift_height_;
    double default_L_, default_W_, default_H_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_server");
    ros::NodeHandle nh("~");

    TrajectoryServer server(nh);

    ros::spin();
    return 0;
}
