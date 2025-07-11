#pragma once

#include "path_attributes.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iomanip>
#include <sstream>

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Enhanced path segment with rich attributes
 * 
 * This replaces the simple std::pair<Pose, Pose> with a structure
 * that can carry rich attribute information for path planning.
 */
struct EnhancedPathSegment {
  // ========================================================================
  // CORE SEGMENT DATA
  // ========================================================================
  
  /// Start pose (x, y, theta)
  Eigen::Vector3d start_pose;
  
  /// End pose (x, y, theta)
  Eigen::Vector3d end_pose;
  
  /// Segment attributes (velocity, constraints, etc.)
  PathAttributes attributes;
  
  /// Segment ID for traceability (optional)
  std::optional<int32_t> segment_id;
  
  /// Source link ID that generated this segment (optional)
  std::optional<int32_t> source_link_id;
  
  /// Source node IDs that this segment connects (optional)
  std::optional<std::pair<int32_t, int32_t>> source_node_ids;
  
  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================
  
  /**
   * @brief Default constructor
   */
  EnhancedPathSegment() = default;
  
  /**
   * @brief Basic constructor with poses
   */
  EnhancedPathSegment(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
    : start_pose(start), end_pose(end) {}
  
  /**
   * @brief Full constructor with attributes
   */
  EnhancedPathSegment(const Eigen::Vector3d& start, 
                      const Eigen::Vector3d& end,
                      const PathAttributes& attrs)
    : start_pose(start), end_pose(end), attributes(attrs) {}
  
  /**
   * @brief Constructor with link traceability
   */
  EnhancedPathSegment(const Eigen::Vector3d& start, 
                      const Eigen::Vector3d& end,
                      const PathAttributes& attrs,
                      int32_t link_id,
                      const std::pair<int32_t, int32_t>& node_ids)
    : start_pose(start), end_pose(end), attributes(attrs), 
      source_link_id(link_id), source_node_ids(node_ids) {}
  
  // ========================================================================
  // GEOMETRIC METHODS
  // ========================================================================
  
  /**
   * @brief Calculate segment length
   */
  double getLength() const {
    Eigen::Vector2d start_pos = start_pose.head<2>();
    Eigen::Vector2d end_pos = end_pose.head<2>();
    return (end_pos - start_pos).norm();
  }
  
  /**
   * @brief Calculate angle change (delta theta)
   */
  double getAngleChange() const {
    double start_angle = normalizeAngle(start_pose(2));
    double end_angle = normalizeAngle(end_pose(2));
    double angle_diff = end_angle - start_angle;
    
    // Normalize to [-π, π]
    if (angle_diff > M_PI) {
      angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }
    
    return angle_diff;
  }
  
  /**
   * @brief Calculate curvature (angle change per unit length)
   */
  double getCurvature() const {
    double length = getLength();
    if (length < 1e-6) return 0.0;
    return std::abs(getAngleChange()) / length;
  }
  
  /**
   * @brief Interpolate pose at parameter t ∈ [0, 1]
   */
  Eigen::Vector3d interpolate(double t) const {
    t = std::clamp(t, 0.0, 1.0);
    
    // Linear interpolation for position
    Eigen::Vector2d start_pos = start_pose.head<2>();
    Eigen::Vector2d end_pos = end_pose.head<2>();
    Eigen::Vector2d interp_pos = start_pos + t * (end_pos - start_pos);
    
    // Interpolate angle (handling wrap-around)
    double start_angle = normalizeAngle(start_pose(2));
    double end_angle = normalizeAngle(end_pose(2));
    double angle_diff = getAngleChange();
    double interp_angle = normalizeAngle(start_angle + t * angle_diff);
    
    return Eigen::Vector3d(interp_pos.x(), interp_pos.y(), interp_angle);
  }
  
  /**
   * @brief Generate interpolated poses with given resolution
   */
  std::vector<Eigen::Vector3d> interpolateWithResolution(double resolution) const {
    std::vector<Eigen::Vector3d> poses;
    
    double length = getLength();
    int num_steps = std::max(1, static_cast<int>(length / resolution));
    
    for (int i = 0; i <= num_steps; ++i) {
      double t = static_cast<double>(i) / num_steps;
      poses.push_back(interpolate(t));
    }
    
    return poses;
  }
  
  // ========================================================================
  // CONVERSION METHODS
  // ========================================================================
  
  /**
   * @brief Convert to legacy PathSegment format
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> toLegacySegment() const {
    return {start_pose, end_pose};
  }
  
  /**
   * @brief Convert to VPath with attribute-aware velocity
   */
  std::vector<std::pair<Eigen::Vector3d, double>> toVPathPoses(double resolution = 0.02) const {
    std::vector<std::pair<Eigen::Vector3d, double>> vpath_poses;
    
    auto interpolated_poses = interpolateWithResolution(resolution);
    double velocity = attributes.getEffectiveVelocity();
    
    for (const auto& pose : interpolated_poses) {
      vpath_poses.emplace_back(pose, velocity);
    }
    
    return vpath_poses;
  }
  
  // ========================================================================
  // ATTRIBUTE MANAGEMENT
  // ========================================================================
  
  /**
   * @brief Apply velocity constraint based on curvature
   */
  void applyVelocityConstraints(double max_curvature_velocity = 0.3) {
    double curvature = getCurvature();
    
    if (curvature > 0.1) { // High curvature
      double curvature_velocity = std::max(max_curvature_velocity, 
                                         attributes.getEffectiveVelocity() * 0.5);
      
      if (!attributes.target_velocity.has_value() || 
          attributes.target_velocity.value() > curvature_velocity) {
        attributes.target_velocity = curvature_velocity;
      }
    }
  }
  
  /**
   * @brief Apply attribute from source link
   */
  void applyLinkAttributes(const PathAttributes& link_attrs) {
    attributes = attributes.merge(link_attrs);
  }
  
  // ========================================================================
  // UTILITY METHODS
  // ========================================================================
  
  /**
   * @brief Get segment summary string
   */
  std::string toString() const {
    std::ostringstream oss;
    oss << "Segment{";
    oss << "length:" << std::fixed << std::setprecision(3) << getLength() << "m ";
    oss << "angle_change:" << std::fixed << std::setprecision(1) 
        << (getAngleChange() * 180.0 / M_PI) << "° ";
    
    if (source_link_id.has_value()) {
      oss << "link:" << source_link_id.value() << " ";
    }
    
    oss << attributes.toString();
    oss << "}";
    return oss.str();
  }
  
private:
  /**
   * @brief Normalize angle to [-π, π]
   */
  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }
};

/**
 * @brief Collection of enhanced path segments
 */
using EnhancedPathSegments = std::vector<EnhancedPathSegment>;

/**
 * @brief Utility functions for enhanced path segments
 */
namespace enhanced_path_utils {

/**
 * @brief Convert legacy path segments to enhanced format
 */
EnhancedPathSegments fromLegacySegments(
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& legacy_segments,
  const PathAttributes& default_attributes = PathAttributes{});

/**
 * @brief Convert enhanced segments back to legacy format
 */
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> toLegacySegments(
  const EnhancedPathSegments& enhanced_segments);

/**
 * @brief Calculate total path length
 */
double calculateTotalLength(const EnhancedPathSegments& segments);

/**
 * @brief Calculate estimated travel time
 */
double calculateTotalTime(const EnhancedPathSegments& segments);

/**
 * @brief Apply velocity smoothing across segments
 */
EnhancedPathSegments applySmoothVelocity(const EnhancedPathSegments& segments,
                                        double max_acceleration = 1.0);

/**
 * @brief Generate VPath from enhanced segments
 */
std::vector<std::pair<Eigen::Vector3d, double>> generateVPath(
  const EnhancedPathSegments& segments, 
  double resolution = 0.02);

} // namespace enhanced_path_utils

} // namespace data_structures  
} // namespace vrobot_route_follow