#ifndef TRAJECTORY_GENERATOR_TYPES_H
#define TRAJECTORY_GENERATOR_TYPES_H

#include <Eigen/Dense>
#include <vector>

namespace trajectory_generator {

struct Point2D {
    double x;  // mm in pattern coordinate
    double y;  // mm in pattern coordinate

    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Polyline {
    std::vector<Point2D> points;  // continuous polyline vertices
    bool is_closed;               // true = closed contour (first == last implicitly)

    Polyline() : is_closed(false) {}
};

struct FaceFrame {
    Eigen::Vector3d origin;   // face center in base frame
    Eigen::Vector3d u_axis;   // in-plane horizontal direction
    Eigen::Vector3d v_axis;   // in-plane vertical direction (points up for side faces)
    Eigen::Vector3d normal;   // outward normal (pen should point along -normal)
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR_TYPES_H
