#include "vrobot_route_follow/data_structures/curve_link_info.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include <cmath>
#include <algorithm>

namespace vrobot_route_follow {
namespace data_structures {

// ========================================================================
// BÉZIER CURVE EVALUATION
// ========================================================================

Eigen::Vector2d CurveLinkInfo::evaluateBezier(double t, 
                                               const Eigen::Vector2d& start_pos, 
                                               const Eigen::Vector2d& end_pos) const {
    // Clamp t to [0, 1]
    t = std::clamp(t, 0.0, 1.0);
    
    // Cubic Bézier formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;
    
    Eigen::Vector2d result = 
        mt3 * start_pos +
        3.0 * mt2 * t * control_point_1 +
        3.0 * mt * t2 * control_point_2 +
        t3 * end_pos;
    
    return result;
}

Eigen::Vector2d CurveLinkInfo::evaluateBezierTangent(double t,
                                                     const Eigen::Vector2d& start_pos,
                                                     const Eigen::Vector2d& end_pos) const {
    // First derivative of cubic Bézier curve
    // B'(t) = 3(1-t)²(P₁-P₀) + 6(1-t)t(P₂-P₁) + 3t²(P₃-P₂)
    t = std::clamp(t, 0.0, 1.0);
    
    double t2 = t * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    
    Eigen::Vector2d tangent = 
        3.0 * mt2 * (control_point_1 - start_pos) +
        6.0 * mt * t * (control_point_2 - control_point_1) +
        3.0 * t2 * (end_pos - control_point_2);
    
    return tangent;
}

double CurveLinkInfo::calculateCurvatureAt(double t,
                                           const Eigen::Vector2d& start_pos,
                                           const Eigen::Vector2d& end_pos) const {
    // Calculate first and second derivatives
    Eigen::Vector2d first_deriv = evaluateBezierTangent(t, start_pos, end_pos);
    
    // Second derivative of cubic Bézier
    // B''(t) = 6(1-t)(P₂-2P₁+P₀) + 6t(P₃-2P₂+P₁)
    double mt = 1.0 - t;
    Eigen::Vector2d second_deriv = 
        6.0 * mt * (control_point_2 - 2.0 * control_point_1 + start_pos) +
        6.0 * t * (end_pos - 2.0 * control_point_2 + control_point_1);
    
    // Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
    double cross_product = first_deriv.x() * second_deriv.y() - first_deriv.y() * second_deriv.x();
    double speed_squared = first_deriv.squaredNorm();
    
    if (speed_squared < 1e-9) {
        return 0.0; // Undefined curvature at stationary points
    }
    
    return std::abs(cross_product) / std::pow(speed_squared, 1.5);
}

// ========================================================================
// CURVE ANALYSIS
// ========================================================================

double CurveLinkInfo::findMaxCurvature(const Eigen::Vector2d& start_pos,
                                       const Eigen::Vector2d& end_pos,
                                       int samples) const {
    double max_curvature = 0.0;
    
    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        double curvature = calculateCurvatureAt(t, start_pos, end_pos);
        max_curvature = std::max(max_curvature, curvature);
    }
    
    return max_curvature;
}

double CurveLinkInfo::estimateCurveLength(const Eigen::Vector2d& start_pos,
                                          const Eigen::Vector2d& end_pos,
                                          double tolerance) const {
    // Adaptive subdivision for length estimation
    std::function<double(double, double, int)> subdivide = 
        [&](double t1, double t2, int depth) -> double {
        
        if (depth > 20) return 0.0; // Prevent infinite recursion
        
        Eigen::Vector2d p1 = evaluateBezier(t1, start_pos, end_pos);
        Eigen::Vector2d p2 = evaluateBezier(t2, start_pos, end_pos);
        Eigen::Vector2d pm = evaluateBezier((t1 + t2) / 2.0, start_pos, end_pos);
        
        double chord_length = (p2 - p1).norm();
        double polyline_length = (pm - p1).norm() + (p2 - pm).norm();
        
        if (std::abs(polyline_length - chord_length) < tolerance) {
            return polyline_length;
        } else {
            double mid = (t1 + t2) / 2.0;
            return subdivide(t1, mid, depth + 1) + subdivide(mid, t2, depth + 1);
        }
    };
    
    return subdivide(0.0, 1.0, 0);
}

std::pair<double, Eigen::Vector2d> CurveLinkInfo::findClosestPoint(
    const Eigen::Vector2d& query_point,
    const Eigen::Vector2d& start_pos,
    const Eigen::Vector2d& end_pos) const {
    
    // Newton-Raphson method for finding closest point
    double t = 0.5; // Initial guess
    const int max_iterations = 20;
    const double tolerance = 1e-6;
    
    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Vector2d curve_point = evaluateBezier(t, start_pos, end_pos);
        Eigen::Vector2d tangent = evaluateBezierTangent(t, start_pos, end_pos);
        
        // Function f(t) = (C(t) - Q) · C'(t) = 0 (where C is curve, Q is query point)
        Eigen::Vector2d diff = curve_point - query_point;
        double f = diff.dot(tangent);
        
        if (std::abs(f) < tolerance) {
            break;
        }
        
        // Second derivative for Newton's method
        double mt = 1.0 - t;
        Eigen::Vector2d second_deriv = 
            6.0 * mt * (control_point_2 - 2.0 * control_point_1 + start_pos) +
            6.0 * t * (end_pos - 2.0 * control_point_2 + control_point_1);
        
        double f_prime = tangent.squaredNorm() + diff.dot(second_deriv);
        
        if (std::abs(f_prime) < 1e-9) {
            break; // Avoid division by zero
        }
        
        t = t - f / f_prime;
        t = std::clamp(t, 0.0, 1.0);
    }
    
    Eigen::Vector2d closest_point = evaluateBezier(t, start_pos, end_pos);
    return {t, closest_point};
}

// ========================================================================
// PATH GENERATION
// ========================================================================

std::vector<Eigen::Vector3d> CurveLinkInfo::interpolateCurve(
    const Eigen::Vector3d& start_pose,
    const Eigen::Vector3d& end_pose,
    double resolution) const {
    
    std::vector<Eigen::Vector3d> poses;
    
    Eigen::Vector2d start_pos = start_pose.head<2>();
    Eigen::Vector2d end_pos = end_pose.head<2>();
    
    // Estimate curve length to determine number of samples
    double length = estimateCurveLength(start_pos, end_pos);
    int num_samples = std::max(2, static_cast<int>(length / resolution));
    
    poses.reserve(num_samples);
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        
        // Get position from Bézier curve
        Eigen::Vector2d pos = evaluateBezier(t, start_pos, end_pos);
        
        // Get orientation from tangent vector
        Eigen::Vector2d tangent = evaluateBezierTangent(t, start_pos, end_pos);
        double theta = std::atan2(tangent.y(), tangent.x());
        
        poses.emplace_back(pos.x(), pos.y(), theta);
    }
    
    // Ensure we end exactly at the target pose
    if (!poses.empty()) {
        poses.back() = end_pose;
    }
    
    return poses;
}

// ========================================================================
// VALIDATION
// ========================================================================

bool CurveLinkInfo::isValidCurve(const Eigen::Vector2d& start_pos,
                                 const Eigen::Vector2d& end_pos) const {
    // Check for reasonable control point positions
    Eigen::Vector2d start_to_end = end_pos - start_pos;
    double baseline_length = start_to_end.norm();
    
    if (baseline_length < 1e-6) {
        return false; // Start and end too close
    }
    
    // Check control points are not too far from baseline
    double max_deviation = baseline_length * 2.0; // Allow 2x baseline length deviation
    
    double cp1_dist = (control_point_1 - start_pos).norm();
    double cp2_dist = (control_point_2 - end_pos).norm();
    
    if (cp1_dist > max_deviation || cp2_dist > max_deviation) {
        return false;
    }
    
    // Check maximum curvature is reasonable
    double max_curv = findMaxCurvature(start_pos, end_pos);
    return max_curv < 10.0; // Reasonable curvature limit
}

bool CurveLinkInfo::hasExcessiveCurvature(const Eigen::Vector2d& start_pos,
                                          const Eigen::Vector2d& end_pos,
                                          double max_curvature_limit) const {
    double max_curv = findMaxCurvature(start_pos, end_pos);
    return max_curv > max_curvature_limit;
}

// ========================================================================
// CONVERSION UTILITIES  
// ========================================================================

LinkInfo CurveLinkInfo::toLinkInfo() const {
    LinkInfo link;
    link.id_straight_link = id_curve_link; // Use same ID
    link.id_start = id_start;
    link.id_end = id_end;
    link.map_id = map_id;
    link.bidirectional = bidirectional;
    
    // Estimate properties for straight link approximation
    if (curve_length.has_value()) {
        link.distance = curve_length.value();
    }
    
    if (max_velocity.has_value()) {
        link.max_velocity = max_velocity.value();
    }
    
    return link;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> CurveLinkInfo::getRelativeControlPoints(
    const Eigen::Vector2d& start_pos,
    const Eigen::Vector2d& end_pos) const {
    
    // Convert control points to relative coordinates (0 to 1) along baseline
    Eigen::Vector2d baseline = end_pos - start_pos;
    
    if (baseline.norm() < 1e-9) {
        return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }
    
    Eigen::Vector2d cp1_rel = control_point_1 - start_pos;
    Eigen::Vector2d cp2_rel = control_point_2 - start_pos;
    
    // Project onto baseline direction
    double baseline_length = baseline.norm();
    
    return {cp1_rel, cp2_rel};
}

} // namespace data_structures
} // namespace vrobot_route_follow