#pragma once

#include <Eigen/Dense>
#include <vector>
#include <optional>

#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"

namespace vrobot_route_follow {

/**
 * @brief Geometric utilities for rich graph operations
 * 
 * This class provides geometric utilities specifically designed for the rich graph
 * architecture, including advanced projection, interpolation, and spatial queries.
 */
class RichGeometricUtils {
public:
    // ========================================================================
    // DISTANCE CALCULATIONS
    // ========================================================================
    
    /**
     * @brief Calculate Euclidean distance between two 2D points
     */
    static double euclideanDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    
    /**
     * @brief Calculate distance between two poses (ignoring orientation)
     */
    static double poseDistance(const Eigen::Vector3d& pose1, const Eigen::Vector3d& pose2);
    
    /**
     * @brief Calculate distance between two nodes
     */
    static double nodeDistance(const data_structures::NodeInfo& node1, const data_structures::NodeInfo& node2);
    
    /**
     * @brief Calculate angular difference between two angles (in radians)
     * Result is normalized to [-pi, pi]
     */
    static double angularDifference(double angle1, double angle2);
    
    // ========================================================================
    // POINT-TO-LINE PROJECTIONS
    // ========================================================================
    
    /**
     * @brief Project a point onto a line segment
     * @param point The point to project
     * @param line_start Start point of the line segment
     * @param line_end End point of the line segment
     * @return Projection point and distance to line
     */
    struct ProjectionResult {
        Eigen::Vector2d projection_point;
        double distance_to_line;
        double parameter_t; // Parameter along line segment [0, 1]
        bool is_on_segment; // True if projection falls within segment
    };
    
    static ProjectionResult projectPointToLineSegment(
        const Eigen::Vector2d& point,
        const Eigen::Vector2d& line_start,
        const Eigen::Vector2d& line_end
    );
    
    /**
     * @brief Project a pose onto a link
     */
    static ProjectionResult projectPoseToLink(
        const Eigen::Vector3d& pose,
        const data_structures::NodeInfo& start_node,
        const data_structures::NodeInfo& end_node
    );
    
    /**
     * @brief Find the closest point on a link to a given pose
     */
    static Eigen::Vector3d findClosestPointOnLink(
        const Eigen::Vector3d& pose,
        const data_structures::LinkInfo& link,
        const data_structures::NodeInfo& start_node,
        const data_structures::NodeInfo& end_node
    );
    
    // ========================================================================
    // INTERPOLATION AND PATH GENERATION
    // ========================================================================
    
    /**
     * @brief Interpolate between two poses with smooth orientation
     */
    static Eigen::Vector3d interpolatePoses(
        const Eigen::Vector3d& start_pose,
        const Eigen::Vector3d& end_pose,
        double t
    );
    
    /**
     * @brief Generate interpolated poses along a link
     */
    static std::vector<Eigen::Vector3d> interpolateAlongLink(
        const data_structures::NodeInfo& start_node,
        const data_structures::NodeInfo& end_node,
        double resolution = 0.02
    );
    
    /**
     * @brief Generate smooth path with curvature constraints
     */
    static std::vector<Eigen::Vector3d> generateSmoothPath(
        const std::vector<data_structures::NodeInfo>& nodes,
        double resolution = 0.02,
        double max_curvature = 1.0
    );
    
    // ========================================================================
    // SPATIAL QUERIES
    // ========================================================================
    
    /**
     * @brief Find nodes within a certain radius of a pose
     */
    static std::vector<int32_t> findNodesInRadius(
        const Eigen::Vector3d& center_pose,
        double radius,
        const std::vector<data_structures::NodeInfo>& nodes
    );
    
    /**
     * @brief Find links that intersect with a circle
     */
    static std::vector<int32_t> findLinksInRadius(
        const Eigen::Vector3d& center_pose,
        double radius,
        const std::vector<data_structures::LinkInfo>& links,
        const std::unordered_map<int32_t, data_structures::NodeInfo>& nodes
    );
    
    /**
     * @brief Check if two line segments intersect
     */
    static bool doLineSegmentsIntersect(
        const Eigen::Vector2d& p1, const Eigen::Vector2d& q1,
        const Eigen::Vector2d& p2, const Eigen::Vector2d& q2
    );
    
    // ========================================================================
    // GEOMETRIC ANALYSIS
    // ========================================================================
    
    /**
     * @brief Calculate the curvature at a point given three consecutive poses
     */
    static double calculateCurvature(
        const Eigen::Vector3d& prev_pose,
        const Eigen::Vector3d& current_pose,
        const Eigen::Vector3d& next_pose
    );
    
    /**
     * @brief Calculate the turning angle between two consecutive links
     */
    static double calculateTurningAngle(
        const data_structures::NodeInfo& node1,
        const data_structures::NodeInfo& node2,
        const data_structures::NodeInfo& node3
    );
    
    /**
     * @brief Check if a path has excessive turning
     */
    static bool hasExcessiveTurning(
        const std::vector<data_structures::NodeInfo>& nodes,
        double max_turning_angle = M_PI / 2.0
    );
    
    // ========================================================================
    // COORDINATE TRANSFORMATIONS
    // ========================================================================
    
    /**
     * @brief Convert from local coordinate system to global
     */
    static Eigen::Vector2d localToGlobal(
        const Eigen::Vector2d& local_point,
        const Eigen::Vector3d& reference_pose
    );
    
    /**
     * @brief Convert from global coordinate system to local
     */
    static Eigen::Vector2d globalToLocal(
        const Eigen::Vector2d& global_point,
        const Eigen::Vector3d& reference_pose
    );
    
    /**
     * @brief Transform a pose by a given transformation
     */
    static Eigen::Vector3d transformPose(
        const Eigen::Vector3d& pose,
        const Eigen::Vector3d& transformation
    );
    
    // ========================================================================
    // BOUNDING BOX OPERATIONS
    // ========================================================================
    
    /**
     * @brief Calculate bounding box for a set of nodes
     */
    struct BoundingBox {
        Eigen::Vector2d min_corner;
        Eigen::Vector2d max_corner;
        
        bool contains(const Eigen::Vector2d& point) const {
            return point.x() >= min_corner.x() && point.x() <= max_corner.x() &&
                   point.y() >= min_corner.y() && point.y() <= max_corner.y();
        }
        
        double width() const { return max_corner.x() - min_corner.x(); }
        double height() const { return max_corner.y() - min_corner.y(); }
        Eigen::Vector2d center() const { return (min_corner + max_corner) / 2.0; }
    };
    
    static BoundingBox calculateBoundingBox(
        const std::vector<data_structures::NodeInfo>& nodes
    );
    
    /**
     * @brief Check if two bounding boxes intersect
     */
    static bool doBoundingBoxesIntersect(
        const BoundingBox& box1,
        const BoundingBox& box2
    );
    
    // ========================================================================
    // UTILITY FUNCTIONS
    // ========================================================================
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle);
    
    /**
     * @brief Convert degrees to radians
     */
    static double degToRad(double degrees);
    
    /**
     * @brief Convert radians to degrees
     */
    static double radToDeg(double radians);
    
    /**
     * @brief Check if a value is approximately equal to another (with tolerance)
     */
    static bool isApproximatelyEqual(double a, double b, double tolerance = 1e-9);
    
    /**
     * @brief Clamp a value between min and max
     */
    static double clamp(double value, double min_val, double max_val);
    
    /**
     * @brief Linear interpolation between two values
     */
    static double lerp(double a, double b, double t);

private:
    // Private constructor to prevent instantiation
    RichGeometricUtils() = default;
    
    // Helper functions for internal calculations
    static double cross2D(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
    static int orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r);
    static bool onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r);
};

} // namespace vrobot_route_follow