#ifndef TRAJECTORY_GENERATOR_SVG_READER_H
#define TRAJECTORY_GENERATOR_SVG_READER_H

#include <trajectory_generator/types.h>
#include <string>
#include <vector>

namespace trajectory_generator {

/**
 * @brief Reads SVG files and converts path data to 2D polylines.
 *
 * Supports a subset of SVG path commands: M (move), L (line), C (cubic bezier).
 * Uses a simple hand-rolled parser; no external SVG library required.
 * Coordinates are output in mm, centered at pattern origin.
 */
class SvgReader {
public:
    /**
     * @brief Load SVG from file and convert to polylines.
     * @param path        File path to SVG
     * @param target_width_mm   Desired pattern width in mm
     * @param target_height_mm  Desired pattern height in mm
     * @return vector of polylines (each "M" starts a new polyline)
     */
    std::vector<Polyline> loadFromFile(const std::string& path,
                                       double target_width_mm,
                                       double target_height_mm);

    /**
     * @brief Create polylines directly from point sequences (no SVG file).
     * @param paths  Each inner vector is one continuous polyline
     * @return vector of polylines
     */
    std::vector<Polyline> loadFromPoints(
        const std::vector<std::vector<Point2D>>& paths);

    /**
     * @brief Generate a predefined test pattern (square frame).
     * @param width_mm   Pattern width in mm
     * @param height_mm  Pattern height in mm
     * @return single closed rectangular polyline
     */
    std::vector<Polyline> generateTestSquare(double width_mm, double height_mm);

    /**
     * @brief Generate a circle pattern approximated by line segments.
     * @param radius_mm  Circle radius in mm
     * @param segments   Number of line segments (default 72)
     * @return single circular polyline
     */
    std::vector<Polyline> generateTestCircle(double radius_mm, int segments = 72);

private:
    /**
     * @brief Parse SVG path data string ("d" attribute).
     * @param d_string  SVG path data (e.g., "M 10 10 L 20 20 ...")
     * @param polylines Output polylines
     */
    void parseSvgPathData(const std::string& d_string,
                          std::vector<Polyline>& polylines);

    /**
     * @brief Sample a cubic bezier curve into line segments.
     * @param p0, p1, p2, p3  Control points
     * @param out_points       Output sampled points
     * @param segments         Number of segments
     */
    void sampleCubicBezier(const Point2D& p0, const Point2D& p1,
                           const Point2D& p2, const Point2D& p3,
                           std::vector<Point2D>& out_points,
                           int segments = 20);

    /**
     * @brief Scale and center polylines to fit target dimensions.
     */
    void normalizePolylines(std::vector<Polyline>& polylines,
                            double target_width_mm, double target_height_mm,
                            double svg_min_x, double svg_min_y,
                            double svg_width, double svg_height);
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR_SVG_READER_H
