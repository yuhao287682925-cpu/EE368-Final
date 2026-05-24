#include <trajectory_generator/surface_projector.h>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace trajectory_generator {

SurfaceProjector::SurfaceProjector()
    : L_(0), W_(0), H_(0)
    , block_pose_set_(false)
    , block_size_set_(false)
{
    for (int i = 0; i < 5; ++i) {
        face_offsets_u_[i] = 0.0;
        face_offsets_v_[i] = 0.0;
    }
}

void SurfaceProjector::setBlockPose(const Eigen::Isometry3d& T_block_base) {
    T_block_base_ = T_block_base;
    block_pose_set_ = true;
    if (block_size_set_) {
        updateFaceFrames();
    }
}

void SurfaceProjector::setBlockSize(double L, double W, double H) {
    L_ = L;
    W_ = W;
    H_ = H;
    block_size_set_ = true;
    if (block_pose_set_) {
        updateFaceFrames();
    }
}

void SurfaceProjector::setFaceOffset(int face_id, double u_offset_mm, double v_offset_mm) {
    if (face_id < 0 || face_id > 4) {
        throw std::out_of_range("face_id must be in [0, 4]");
    }
    face_offsets_u_[face_id] = u_offset_mm;
    face_offsets_v_[face_id] = v_offset_mm;
}

FaceFrame SurfaceProjector::getFaceFrame(int face_id) const {
    if (face_id < 0 || face_id > 4) {
        throw std::out_of_range("face_id must be in [0, 4]");
    }
    return face_frames_[face_id];
}

void SurfaceProjector::updateFaceFrames() {
    // Block-local face frames (origin at block center = bottom center)
    // Top face:    origin=(0, 0, H/2), u=(1,0,0), v=(0,1,0), normal=(0,0,1)
    // Front face:  origin=(0, W/2, 0), u=(1,0,0), v=(0,0,1), normal=(0,1,0)
    // Right face:  origin=(L/2, 0, 0), u=(0,1,0), v=(0,0,1), normal=(1,0,0)
    // Back face:   origin=(0, -W/2, 0), u=(-1,0,0), v=(0,0,1), normal=(0,-1,0)
    // Left face:   origin=(-L/2, 0, 0), u=(0,-1,0), v=(0,0,1), normal=(-1,0,0)

    struct LocalFrame {
        Eigen::Vector3d origin;
        Eigen::Vector3d u, v, normal;
    };

    LocalFrame locals[5] = {
        { {0, 0,  H_/2},  {1, 0, 0},  {0, 1, 0},  {0, 0, 1}  },  // Top
        { {0, W_/2, 0},   {1, 0, 0},  {0, 0, 1},  {0, 1, 0}  },  // Front
        { {L_/2, 0, 0},   {0, 1, 0},  {0, 0, 1},  {1, 0, 0}  },  // Right
        { {0, -W_/2, 0},  {-1, 0, 0}, {0, 0, 1},  {0, -1, 0} },  // Back
        { {-L_/2, 0, 0},  {0, -1, 0}, {0, 0, 1},  {-1, 0, 0} },  // Left
    };

    for (int i = 0; i < 5; ++i) {
        face_frames_[i].origin = T_block_base_ * locals[i].origin;
        face_frames_[i].u_axis = T_block_base_.linear() * locals[i].u;
        face_frames_[i].v_axis = T_block_base_.linear() * locals[i].v;
        face_frames_[i].normal = T_block_base_.linear() * locals[i].normal;
    }
}

block_drawing_msgs::SurfaceTrajectory SurfaceProjector::project2DToFace(
    const Polyline& polyline,
    int face_id,
    double pen_tip_offset) {

    if (!block_pose_set_ || !block_size_set_) {
        throw std::runtime_error("Block pose and size must be set before projection");
    }

    FaceFrame F = getFaceFrame(face_id);

    // Apply face offset: pattern (0,0) → face_origin + u_offset*u + v_offset*v
    double u_off_m = face_offsets_u_[face_id] / 1000.0;
    double v_off_m = face_offsets_v_[face_id] / 1000.0;
    Eigen::Vector3d anchored_origin = F.origin + u_off_m * F.u_axis + v_off_m * F.v_axis;

    block_drawing_msgs::SurfaceTrajectory traj;
    traj.face_id = face_id;

    if (polyline.points.empty()) return traj;

    traj.waypoints.reserve(polyline.points.size());
    traj.arc_lengths.reserve(polyline.points.size());

    double arc_length = 0.0;
    Eigen::Vector3d prev_position(0, 0, 0);
    bool first_point = true;

    for (size_t i = 0; i < polyline.points.size(); ++i) {
        const auto& p2d = polyline.points[i];

        // Convert mm to m for ROS
        double x_m = p2d.x / 1000.0;
        double y_m = p2d.y / 1000.0;

        // Position on surface: anchored_origin + x*u + y*v
        Eigen::Vector3d P_on_surface = anchored_origin + x_m * F.u_axis + y_m * F.v_axis;

        // Compute travel direction (tangent)
        Eigen::Vector3d travel_dir;
        if (i < polyline.points.size() - 1) {
            const auto& next = polyline.points[i + 1];
            travel_dir = ((anchored_origin + next.x/1000.0 * F.u_axis + next.y/1000.0 * F.v_axis) - P_on_surface).normalized();
        } else if (i > 0) {
            travel_dir = (P_on_surface - prev_position).normalized();
        } else {
            travel_dir = F.u_axis;  // default: along u-axis
        }

        // Pen orientation: Z=-normal, X=tangent
        Eigen::Quaterniond R_pen = computePenOrientation(travel_dir, F.normal);

        // Compensate for pen tip offset: flange = surface_point - R * (0,0,tip_length)
        Eigen::Vector3d offset(0, 0, pen_tip_offset);
        Eigen::Vector3d P_flange = P_on_surface - R_pen * offset;

        traj.waypoints.push_back(eigenToRosPose(P_flange, R_pen));

        // Arc length
        if (!first_point) {
            arc_length += (P_on_surface - prev_position).norm();
        }
        traj.arc_lengths.push_back(arc_length);
        prev_position = P_on_surface;
        first_point = false;
    }

    return traj;
}

void SurfaceProjector::getFaceBounds(
    int face_id, double& u_min, double& u_max,
    double& v_min, double& v_max) const {

    double L_2 = L_ * 500.0;  // half-length in mm
    double W_2 = W_ * 500.0;
    double H_2 = H_ * 500.0;

    switch (face_id) {
        case 0: u_min = -L_2; u_max = L_2; v_min = -W_2; v_max = W_2; break;  // Top
        case 1: u_min = -L_2; u_max = L_2; v_min = -H_2; v_max = H_2; break;  // Front
        case 2: u_min = -W_2; u_max = W_2; v_min = -H_2; v_max = H_2; break;  // Right
        case 3: u_min = -L_2; u_max = L_2; v_min = -H_2; v_max = H_2; break;  // Back
        case 4: u_min = -W_2; u_max = W_2; v_min = -H_2; v_max = H_2; break;  // Left
    }
}

void SurfaceProjector::resolveFacePoint(int& face_id, double& u_mm, double& v_mm) const {
    // Recursively resolve face crossing (max 2 levels for corner cases)
    for (int iter = 0; iter < 2; ++iter) {
        double u_min, u_max, v_min, v_max;
        getFaceBounds(face_id, u_min, u_max, v_min, v_max);

        // Check bounds
        bool in_u = (u_mm >= u_min && u_mm <= u_max);
        bool in_v = (v_mm >= v_min && v_mm <= v_max);
        if (in_u && in_v) return;  // stays on current face

        // Determine crossing direction and edge
        double excess_u = 0, excess_v = 0;
        if (u_mm > u_max)      excess_u = u_mm - u_max;
        else if (u_mm < u_min) excess_u = u_mm - u_min;
        if (v_mm > v_max)      excess_v = v_mm - v_max;
        else if (v_mm < v_min) excess_v = v_mm - v_min;

        int next_face = -1;
        double new_u = 0, new_v = 0;

        // Top face (0) adjacency:
        //   v > v_max → Front(1):  u'=u,       v'=H/2 - (v-v_max)
        //   v < v_min → Back(3):   u'=-u,      v'=H/2 + v  (since v < v_min = -W/2)
        //   u > u_max → Right(2):  u'=-v,      v'=H/2 - (u-u_max)
        //   u < u_min → Left(4):   u'=v,       v'=H/2 + u  (since u < u_min = -L/2)
        if (face_id == 0) {
            double H_2 = H_ * 500.0;
            if (v_mm > v_max) {
                next_face = 1;  // Front
                new_u = u_mm;
                new_v = v_max + excess_v;  // H_2 + excess... wait
                // Actually: snap to edge, then continue on front
                // v goes past v_max=W/2 into front face region
                // On front: v extends from +H/2 downward
                // So v' = H_2 - (v_mm - v_max)
                new_u = u_mm;
                new_v = H_2 - std::abs(excess_v);
            } else if (v_mm < v_min) {
                next_face = 3;  // Back
                new_u = -u_mm;
                new_v = H_2 + excess_v;  // H_2 - |excess| (excess_v is negative)
                // v < v_min: v_mm = v_min + excess_v where excess_v < 0
                // On back: u' = -u, v' = H_2 + excess_v (starts at H_2, goes down)
                new_u = -u_mm;
                new_v = H_2 + excess_v;
            } else if (u_mm > u_max) {
                next_face = 2;  // Right
                new_u = -(v_mm);  // u' = -v
                new_v = H_2 - std::abs(excess_u);
            } else if (u_mm < u_min) {
                next_face = 4;  // Left
                new_u = v_mm;
                new_v = H_2 + excess_u;  // u < u_min: excess_u < 0
            }
        }
        // Front face (1) adjacency:
        // The front face is on the +W/2 side. Its edges:
        //   v(+H/2) → top edge (shared with Top's +W/2 edge)
        //   v(-H/2) → bottom edge (open, no neighbor)
        //   u(+L/2) → Right(2): u'=W/2 - (u-L/2), v'=v
        //   u(-L/2) → Left(4):  u'=W/2 + (u+L/2), v'=v
        //   v(+H/2) → Top(0):   u'=u,        v'=W/2 + (v-H/2)
        else if (face_id == 1) {
            double W_2 = W_ * 500.0;
            if (v_mm > v_max) {
                next_face = 0;  // Top
                new_u = u_mm;
                new_v = W_2 + excess_v;  // excess_v < 0 from v > v_max... wait
                // v_max = H_2, excess_v = v_mm - H_2 (positive since v_mm > H_2)
                // On top: v' = W_2 - (v_mm - H_2)... no
                // Actually re-thinking: front v=+H/2 ↔ top v=+W/2
                // Excess past H_2 means it crosses onto top face, going "up" on top
                // u' stays same, v' continues past W/2
                new_u = u_mm;
                new_v = W_2 + excess_v;  // exceeds W/2
            } else if (u_mm > u_max) {
                next_face = 2;  // Right
                new_u = W_2 - std::abs(excess_u);  // enter from top of right face
                new_v = v_mm;
            } else if (u_mm < u_min) {
                next_face = 4;  // Left
                new_u = -W_2 + std::abs(excess_u);
                new_v = v_mm;
            }
        }
        // Right face (2) adjacency:
        //   v(+H/2) → Top(0):   u'=L/2 + (v-H/2), v'=-u
        //   u(+W/2) → Front(1): u'=L/2 + (u-W/2), v'=v
        //   u(-W/2) → Back(3):  u'=-L/2 - (u+W/2), v'=v
        else if (face_id == 2) {
            double L_2 = L_ * 500.0;
            if (v_mm > v_max) {
                next_face = 0;  // Top
                new_u = L_2 + excess_v;
                new_v = -(u_mm);
            } else if (u_mm > u_max) {
                next_face = 1;  // Front
                new_u = L_2 + excess_u;
                new_v = v_mm;
            } else if (u_mm < u_min) {
                next_face = 3;  // Back
                new_u = -L_2 + excess_u;
                new_v = v_mm;
            }
        }
        // Back face (3) adjacency:
        //   v(+H/2) → Top(0):   u'=-u, v'=-W/2 - (v-H/2)
        //   u(+L/2) → Right(2): u'=W/2 - (u-L/2), v'=v
        //   u(-L/2) → Left(4):  u'=-W/2 + (u+L/2), v'=v
        else if (face_id == 3) {
            double W2 = W_ * 500.0;
            if (v_mm > v_max) {
                next_face = 0;  // Top
                new_u = -(u_mm);
                new_v = -(W2) + (-excess_v);  // past top's negative v edge
            } else if (u_mm > u_max) {
                next_face = 2;  // Right
                new_u = W2 - std::abs(excess_u);
                new_v = v_mm;
            } else if (u_mm < u_min) {
                next_face = 4;  // Left
                new_u = -W2 + std::abs(excess_u);
                new_v = v_mm;
            }
        }
        // Left face (4) adjacency:
        //   v(+H/2) → Top(0):   u'=-L/2 - (v-H/2), v'=u
        //   u(+W/2) → Front(1): u'=-L/2 - (u-W/2), v'=v
        //   u(-W/2) → Back(3):  u'=L/2 + (u+W/2), v'=v
        else if (face_id == 4) {
            double L2 = L_ * 500.0;
            if (v_mm > v_max) {
                next_face = 0;  // Top
                new_u = -L2 - std::abs(excess_v);
                new_v = u_mm;
            } else if (u_mm > u_max) {
                next_face = 1;  // Front
                new_u = -L2 - std::abs(excess_u);
                new_v = v_mm;
            } else if (u_mm < u_min) {
                next_face = 3;  // Back
                new_u = L2 + std::abs(excess_u);
                new_v = v_mm;
            }
        }

        if (next_face < 0) break;  // can't resolve → clamp
        face_id = next_face;
        u_mm = new_u;
        v_mm = new_v;
    }
}

block_drawing_msgs::SurfaceTrajectory SurfaceProjector::project2DContinuous(
    const Polyline& polyline,
    int center_face,
    double center_u_mm,
    double center_v_mm,
    double pen_tip_offset) {

    if (!block_pose_set_ || !block_size_set_) {
        throw std::runtime_error("Block pose and size must be set before projection");
    }

    block_drawing_msgs::SurfaceTrajectory traj;
    traj.face_id = -1;  // signal: multi-face trajectory

    if (polyline.points.empty()) return traj;

    traj.waypoints.reserve(polyline.points.size());
    traj.arc_lengths.reserve(polyline.points.size());

    double arc_length = 0.0;
    Eigen::Vector3d prev_position(0, 0, 0);
    bool first_point = true;

    for (size_t i = 0; i < polyline.points.size(); ++i) {
        const auto& p2d = polyline.points[i];

        // Target (u,v) on block surface, starting from center face
        int face_id = center_face;
        double u_mm = center_u_mm + p2d.x;
        double v_mm = center_v_mm + p2d.y;

        // Resolve to actual face (may cross boundaries)
        resolveFacePoint(face_id, u_mm, v_mm);

        FaceFrame F = getFaceFrame(face_id);

        // Convert mm to m
        double x_m = u_mm / 1000.0;
        double y_m = v_mm / 1000.0;

        // 3D position on the resolved face
        Eigen::Vector3d P_on_surface = F.origin + x_m * F.u_axis + y_m * F.v_axis;

        // Travel direction
        Eigen::Vector3d travel_dir;
        if (i < polyline.points.size() - 1) {
            // Look ahead to next point for tangent
            int next_face = center_face;
            const auto& next = polyline.points[i + 1];
            double nu = center_u_mm + next.x;
            double nv = center_v_mm + next.y;
            resolveFacePoint(next_face, nu, nv);
            FaceFrame Fn = getFaceFrame(next_face);
            Eigen::Vector3d P_next = Fn.origin + nu/1000.0 * Fn.u_axis + nv/1000.0 * Fn.v_axis;
            travel_dir = (P_next - P_on_surface).normalized();
        } else if (i > 0) {
            travel_dir = (P_on_surface - prev_position).normalized();
        } else {
            travel_dir = F.u_axis;
        }

        // Pen orientation: Z=-normal, X=tangent
        Eigen::Quaterniond R_pen = computePenOrientation(travel_dir, F.normal);

        // Flange compensation
        Eigen::Vector3d offset(0, 0, pen_tip_offset);
        Eigen::Vector3d P_flange = P_on_surface - R_pen * offset;

        traj.waypoints.push_back(eigenToRosPose(P_flange, R_pen));

        if (!first_point) {
            arc_length += (P_on_surface - prev_position).norm();
        }
        traj.arc_lengths.push_back(arc_length);
        prev_position = P_on_surface;
        first_point = false;
    }

    return traj;
}

std::vector<geometry_msgs::Pose> SurfaceProjector::generateTransitionWaypoints(
    int from_face, int to_face,
    const Point2D& last_point_2d,
    const Point2D& first_point_2d,
    double lift_height,
    double pen_tip_offset) {

    FaceFrame F_from = getFaceFrame(from_face);
    FaceFrame F_to = getFaceFrame(to_face);

    // Last point on from-face
    Eigen::Vector3d P_from_surface = F_from.origin
        + last_point_2d.x/1000.0 * F_from.u_axis
        + last_point_2d.y/1000.0 * F_from.v_axis;

    // First point on to-face
    Eigen::Vector3d P_to_surface = F_to.origin
        + first_point_2d.x/1000.0 * F_to.u_axis
        + first_point_2d.y/1000.0 * F_to.v_axis;

    // Travel direction on from-face (last segment direction)
    Eigen::Vector3d from_travel_dir = F_from.u_axis;
    Eigen::Quaterniond R_from = computePenOrientation(from_travel_dir, F_from.normal);

    // Lifted position above last point
    Eigen::Vector3d P_lifted_from = P_from_surface + F_from.normal * lift_height;
    Eigen::Vector3d P_flange_lifted_from = P_lifted_from
        - R_from * Eigen::Vector3d(0, 0, pen_tip_offset);

    // Travel direction on to-face (first segment direction)
    Eigen::Vector3d to_travel_dir = F_to.u_axis;
    Eigen::Quaterniond R_to = computePenOrientation(to_travel_dir, F_to.normal);

    // Lifted position above first point
    Eigen::Vector3d P_lifted_to = P_to_surface + F_to.normal * lift_height;
    Eigen::Vector3d P_flange_lifted_to = P_lifted_to
        - R_to * Eigen::Vector3d(0, 0, pen_tip_offset);

    // Approach position (first point on to-face)
    Eigen::Vector3d P_flange_approach = P_to_surface
        - R_to * Eigen::Vector3d(0, 0, pen_tip_offset);

    std::vector<geometry_msgs::Pose> result;
    result.push_back(eigenToRosPose(P_flange_lifted_from, R_from));
    result.push_back(eigenToRosPose(P_flange_lifted_to, R_to));
    result.push_back(eigenToRosPose(P_flange_approach, R_to));
    return result;
}

Eigen::Quaterniond SurfaceProjector::computePenOrientation(
    const Eigen::Vector3d& travel_direction,
    const Eigen::Vector3d& surface_normal) const {

    // Z-axis = -normal (pen pointing into surface)
    // X-axis = travel direction (projected to surface plane to ensure orthogonality)
    Eigen::Vector3d z_axis = -surface_normal.normalized();

    // Make X perpendicular to Z by projecting travel_dir onto the plane normal to Z
    Eigen::Vector3d x_axis = travel_direction - travel_direction.dot(z_axis) * z_axis;
    if (x_axis.norm() < 1e-10) {
        // Fallback: use u_axis of face or world X
        x_axis = Eigen::Vector3d::UnitX();
        x_axis = x_axis - x_axis.dot(z_axis) * z_axis;
    }
    x_axis.normalize();

    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    return Eigen::Quaterniond(R);
}

geometry_msgs::Pose SurfaceProjector::eigenToRosPose(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) const {

    geometry_msgs::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.w = orientation.w();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    return pose;
}

}  // namespace trajectory_generator
