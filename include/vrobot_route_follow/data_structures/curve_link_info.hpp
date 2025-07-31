#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Enhanced curve link information with Bézier curve support
 * 
 * This structure represents curved links from the database that use
 * cubic Bézier curves defined by start/end nodes and two control points.
 */
struct CurveLinkInfo {
    // ========================================================================
    // BASIC PROPERTIES
    // ========================================================================
    
    /// Unique identifier for the curve link
    int32_t id_curve_link;
    
    /// Start node ID
    int32_t id_start;
    
    /// End node ID  
    int32_t id_end;
    
    /// Map ID this curve link belongs to
    int32_t map_id;
    
    // ========================================================================
    // BÉZIER CURVE CONTROL POINTS
    // ========================================================================
    
    /// First control point (influences curve near start)
    Eigen::Vector2d control_point_1;
    
    /// Second control point (influences curve near end)
    Eigen::Vector2d control_point_2;
    
    // ========================================================================
    // DERIVED PROPERTIES
    // ========================================================================
    
    /// Estimated curve length (calculated)
    std::optional<double> curve_length;
    
    /// Maximum curvature along the path
    std::optional<double> max_curvature;
    
    /// Suggested maximum velocity for this curve
    std::optional<double> max_velocity;
    
    /// Whether this curve can be traversed in both directions
    std::optional<bool> bidirectional = true;
    
    // ========================================================================
    // BÉZIER CURVE METHODS
    // ========================================================================
    
    /**
     * @brief Calculate point on Bézier curve at parameter t ∈ [0,1]
     * Uses cubic Bézier formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
     * Where: P₀=start, P₁=control_point_1, P₂=control_point_2, P₃=end
     */
    Eigen::Vector2d evaluateBezier(double t, 
                                   const Eigen::Vector2d& start_pos, 
                                   const Eigen::Vector2d& end_pos) const;
    
    /**
     * @brief Calculate tangent vector at parameter t
     * Derivative of Bézier curve for smooth orientation
     */
    Eigen::Vector2d evaluateBezierTangent(double t,
                                          const Eigen::Vector2d& start_pos,
                                          const Eigen::Vector2d& end_pos) const;
    
    /**
     * @brief Generate interpolated poses along the curve
     * @param start_pose Start pose (x, y, theta)
     * @param end_pose End pose (x, y, theta)  
     * @param resolution Distance between points (meters)
     * @return Vector of interpolated poses with smooth orientations
     */
    std::vector<Eigen::Vector3d> interpolateCurve(
        const Eigen::Vector3d& start_pose,
        const Eigen::Vector3d& end_pose,
        double resolution = 0.02) const;
    
    /**
     * @brief Calculate curvature at parameter t
     * κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
     */
    double calculateCurvatureAt(double t,
                                const Eigen::Vector2d& start_pos,
                                const Eigen::Vector2d& end_pos) const;
    
    /**
     * @brief Find maximum curvature along the entire curve
     */
    double findMaxCurvature(const Eigen::Vector2d& start_pos,
                            const Eigen::Vector2d& end_pos,
                            int samples = 100) const;
    
    /**
     * @brief Estimate curve length using adaptive subdivision
     */
    double estimateCurveLength(const Eigen::Vector2d& start_pos,
                               const Eigen::Vector2d& end_pos,
                               double tolerance = 0.001) const;
    
    /**
     * @brief Find closest point on curve to a given position
     * @param query_point Point to find closest curve point for
     * @param start_pos Curve start position
     * @param end_pos Curve end position
     * @return Parameter t and closest point
     */
    std::pair<double, Eigen::Vector2d> findClosestPoint(
        const Eigen::Vector2d& query_point,
        const Eigen::Vector2d& start_pos,
        const Eigen::Vector2d& end_pos) const;
    
    // ========================================================================
    // VALIDATION AND QUALITY CHECKS
    // ========================================================================
    
    /**
     * @brief Check if curve is well-formed (no self-intersections, reasonable curvature)
     */
    bool isValidCurve(const Eigen::Vector2d& start_pos,
                      const Eigen::Vector2d& end_pos) const;
    
    /**
     * @brief Check if curve has excessive curvature for vehicle navigation
     */
    bool hasExcessiveCurvature(const Eigen::Vector2d& start_pos,
                               const Eigen::Vector2d& end_pos,
                               double max_curvature_limit = 2.0) const;
    
    // ========================================================================
    // CONVERSION UTILITIES
    // ========================================================================
    
    /**
     * @brief Convert to legacy straight link for fallback
     */
    struct LinkInfo toLinkInfo() const;
    
    /**
     * @brief Convert control points to relative coordinates
     */
    std::pair<Eigen::Vector2d, Eigen::Vector2d> getRelativeControlPoints(
        const Eigen::Vector2d& start_pos,
        const Eigen::Vector2d& end_pos) const;
};

} // namespace data_structures
} // namespace vrobot_route_follow