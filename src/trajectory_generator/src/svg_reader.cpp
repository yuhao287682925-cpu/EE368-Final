#include <trajectory_generator/svg_reader.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace trajectory_generator {

std::vector<Polyline> SvgReader::loadFromFile(const std::string& path,
                                               double target_width_mm,
                                               double target_height_mm) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open SVG file: " + path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    // Extract viewBox or width/height for coordinate scaling
    double svg_w = target_width_mm, svg_h = target_height_mm;
    double svg_min_x = 0, svg_min_y = 0;

    // Try to parse viewBox="min_x min_y width height"
    size_t vb_pos = content.find("viewBox=\"");
    if (vb_pos == std::string::npos) {
        vb_pos = content.find("viewBox='");
    }
    if (vb_pos != std::string::npos) {
        vb_pos = content.find("\"", vb_pos);
        if (vb_pos == std::string::npos) vb_pos = content.find("'", content.find("viewBox='"));
        vb_pos++;
        std::string vb_str = content.substr(vb_pos, content.find("\"", vb_pos) - vb_pos);
        std::stringstream ss(vb_str);
        ss >> svg_min_x >> svg_min_y >> svg_w >> svg_h;
    }

    // Find all <path d="..."/> elements
    std::vector<Polyline> all_polylines;
    size_t search_pos = 0;
    while (true) {
        size_t path_start = content.find("<path", search_pos);
        if (path_start == std::string::npos) break;

        // Find d="..."
        size_t d_pos = content.find("d=\"", path_start);
        if (d_pos == std::string::npos) d_pos = content.find("d='", path_start);
        if (d_pos == std::string::npos || d_pos > content.find("/>", path_start) &&
            d_pos > content.find(">", path_start)) {
            search_pos = path_start + 5;
            continue;
        }

        d_pos = content.find("\"", d_pos);
        if (d_pos == std::string::npos) d_pos = content.find("'", content.find("d='", path_start));
        d_pos++;
        size_t d_end = content.find("\"", d_pos);
        if (d_end == std::string::npos) d_end = content.find("'", d_pos);
        std::string d_string = content.substr(d_pos, d_end - d_pos);

        std::vector<Polyline> path_polylines;
        parseSvgPathData(d_string, path_polylines);
        all_polylines.insert(all_polylines.end(),
                             path_polylines.begin(), path_polylines.end());

        search_pos = d_end + 1;
    }

    if (all_polylines.empty()) {
        throw std::runtime_error("No path data found in SVG file: " + path);
    }

    // Normalize coordinates to target dimensions
    normalizePolylines(all_polylines, target_width_mm, target_height_mm,
                       svg_min_x, svg_min_y, svg_w, svg_h);

    return all_polylines;
}

std::vector<Polyline> SvgReader::loadFromPoints(
    const std::vector<std::vector<Point2D>>& paths) {
    std::vector<Polyline> result;
    result.reserve(paths.size());
    for (const auto& path : paths) {
        Polyline pl;
        pl.points = path;
        pl.is_closed = (!path.empty() &&
                        std::hypot(path.front().x - path.back().x,
                                   path.front().y - path.back().y) < 1e-6);
        result.push_back(pl);
    }
    return result;
}

std::vector<Polyline> SvgReader::generateTestSquare(double width_mm, double height_mm) {
    Polyline pl;
    pl.is_closed = true;
    double hw = width_mm / 2.0;
    double hh = height_mm / 2.0;
    pl.points = {
        Point2D(-hw, -hh),
        Point2D( hw, -hh),
        Point2D( hw,  hh),
        Point2D(-hw,  hh),
        Point2D(-hw, -hh)
    };
    return {pl};
}

std::vector<Polyline> SvgReader::generateTestCircle(double radius_mm, int segments) {
    Polyline pl;
    pl.is_closed = true;
    pl.points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i) {
        double angle = 2.0 * M_PI * i / segments;
        pl.points.emplace_back(radius_mm * std::cos(angle),
                               radius_mm * std::sin(angle));
    }
    return {pl};
}

void SvgReader::parseSvgPathData(const std::string& d_string,
                                  std::vector<Polyline>& polylines) {
    Polyline current_pl;
    Point2D current_point(0, 0);
    Point2D last_cubic_ctrl(0, 0);  // for smooth bezier
    char current_cmd = 0;

    std::string cleaned;
    cleaned.reserve(d_string.size());
    for (char c : d_string) {
        if (c == ',') cleaned += ' ';
        else cleaned += c;
    }

    std::stringstream ss(cleaned);
    std::string token;

    auto parseNum = [&](double& val) -> bool {
        while (ss >> token) {
            // Check if token is a command letter
            if (token.size() == 1 && std::isalpha(static_cast<unsigned char>(token[0]))) {
                ss.seekg(-static_cast<int>(token.size()), std::ios_base::cur);
                return false;
            }
            try {
                val = std::stod(token);
                return true;
            } catch (...) {
                continue;
            }
        }
        return false;
    };

    auto handleMove = [&](bool relative) {
        double x, y;
        if (!parseNum(x) || !parseNum(y)) return;
        if (relative) { x += current_point.x; y += current_point.y; }
        if (!current_pl.points.empty()) {
            polylines.push_back(current_pl);
            current_pl.points.clear();
        }
        current_pl.is_closed = false;
        current_pl.points.emplace_back(x, y);
        current_point = Point2D(x, y);
    };

    auto handleLine = [&](bool relative) {
        double x, y;
        if (!parseNum(x) || !parseNum(y)) return;
        if (relative) { x += current_point.x; y += current_point.y; }
        current_pl.points.emplace_back(x, y);
        current_point = Point2D(x, y);
    };

    auto handleBezier = [&](bool relative, bool smooth) {
        double x1, y1, x2, y2, x, y;
        if (smooth) {
            // Reflect previous control point
            x1 = 2 * current_point.x - last_cubic_ctrl.x;
            y1 = 2 * current_point.y - last_cubic_ctrl.y;
        } else {
            if (!parseNum(x1) || !parseNum(y1)) return;
            if (relative) { x1 += current_point.x; y1 += current_point.y; }
        }
        if (!parseNum(x2) || !parseNum(y2)) return;
        if (relative) { x2 += current_point.x; y2 += current_point.y; }
        if (!parseNum(x) || !parseNum(y)) return;
        if (relative) { x += current_point.x; y += current_point.y; }

        last_cubic_ctrl = Point2D(x2, y2);

        std::vector<Point2D> sampled;
        sampleCubicBezier(current_point,
                          Point2D(x1, y1), Point2D(x2, y2),
                          Point2D(x, y), sampled);
        for (const auto& pt : sampled) {
            current_pl.points.push_back(pt);
        }
        current_point = Point2D(x, y);
    };

    while (ss >> token) {
        if (token.size() == 1 && std::isalpha(static_cast<unsigned char>(token[0]))) {
            current_cmd = token[0];
        } else if (token.size() == 1 && !std::isalpha(static_cast<unsigned char>(token[0]))) {
            // It's a number, put it back
            ss.seekg(-static_cast<int>(token.size()), std::ios_base::cur);
        } else if (token.size() > 1) {
            // Could be a command + number glued together, try to handle
            ss.seekg(-static_cast<int>(token.size()), std::ios_base::cur);
        }

        switch (current_cmd) {
            case 'M': handleMove(false); break;
            case 'm': handleMove(true);  break;
            case 'L': handleLine(false); break;
            case 'l': handleLine(true);  break;
            case 'C': handleBezier(false, false); break;
            case 'c': handleBezier(true, false);  break;
            case 'S': handleBezier(false, true);  break;
            case 's': handleBezier(true, true);   break;
            case 'Z':
            case 'z':
                if (!current_pl.points.empty()) {
                    current_pl.is_closed = true;
                    current_pl.points.push_back(current_pl.points.front());
                    current_point = current_pl.points.front();
                }
                break;
            case 'H': case 'h': {
                double x;
                if (parseNum(x)) {
                    if (current_cmd == 'h') x += current_point.x;
                    current_pl.points.emplace_back(x, current_point.y);
                    current_point = Point2D(x, current_point.y);
                }
                break;
            }
            case 'V': case 'v': {
                double y;
                if (parseNum(y)) {
                    if (current_cmd == 'v') y += current_point.y;
                    current_pl.points.emplace_back(current_point.x, y);
                    current_point = Point2D(current_point.x, y);
                }
                break;
            }
            default: break;
        }
    }

    if (!current_pl.points.empty()) {
        polylines.push_back(current_pl);
    }
}

void SvgReader::sampleCubicBezier(const Point2D& p0, const Point2D& p1,
                                   const Point2D& p2, const Point2D& p3,
                                   std::vector<Point2D>& out_points,
                                   int segments) {
    out_points.reserve(out_points.size() + segments);
    for (int i = 1; i <= segments; ++i) {
        double t = static_cast<double>(i) / segments;
        double t2 = t * t;
        double t3 = t2 * t;
        double u = 1.0 - t;
        double u2 = u * u;
        double u3 = u2 * u;

        double x = u3 * p0.x + 3 * u2 * t * p1.x + 3 * u * t2 * p2.x + t3 * p3.x;
        double y = u3 * p0.y + 3 * u2 * t * p1.y + 3 * u * t2 * p2.y + t3 * p3.y;
        out_points.emplace_back(x, y);
    }
}

void SvgReader::normalizePolylines(std::vector<Polyline>& polylines,
                                    double target_width_mm,
                                    double target_height_mm,
                                    double svg_min_x, double svg_min_y,
                                    double svg_width, double svg_height) {
    if (svg_width <= 0 || svg_height <= 0) return;

    double scale_x = target_width_mm / svg_width;
    double scale_y = target_height_mm / svg_height;
    double scale = std::min(scale_x, scale_y);

    // Center offset: pattern will be centered at (0,0)
    double svg_center_x = svg_min_x + svg_width / 2.0;
    double svg_center_y = svg_min_y + svg_height / 2.0;

    for (auto& pl : polylines) {
        for (auto& pt : pl.points) {
            pt.x = (pt.x - svg_center_x) * scale;
            pt.y = (pt.y - svg_center_y) * scale;
        }
    }
}

}  // namespace trajectory_generator
