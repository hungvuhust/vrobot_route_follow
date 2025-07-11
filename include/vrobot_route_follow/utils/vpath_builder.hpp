#pragma once

#include "vrobot_route_follow/data_structures/enhanced_path_segment.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"
#include <rclcpp/time.hpp>
#include <vrobot_local_planner/msg/path.hpp>

namespace vrobot_route_follow {
namespace utils {

/**
 * @brief Builder class for creating VPath with flexible attributes
 * 
 * This class solves the circular dependency issue and provides 
 * a clean interface for VPath creation with rich attributes.
 */
class VPathBuilder {
public:
  // ========================================================================
  // CONSTRUCTION OPTIONS
  // ========================================================================
  
  struct BuildOptions {
    /// Interpolation resolution (meters)
    double interpolation_resolution = 0.02;
    
    /// Default velocity if none specified (m/s)
    double default_velocity = 0.5;
    
    /// Apply velocity smoothing
    bool apply_velocity_smoothing = true;
    
    /// Maximum acceleration for smoothing (m/s²)
    double max_acceleration = 1.0;
    
    /// Apply curvature-based velocity constraints
    bool apply_curvature_constraints = true;
    
    /// Minimum velocity for high curvature sections (m/s)
    double min_curvature_velocity = 0.2;
    
    /// Frame ID for the path
    std::string frame_id = "map";
    
    /// Timestamp for the path
    rclcpp::Time timestamp = rclcpp::Time(0);
  };
  
  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================
  
  VPathBuilder() = default;
  
  explicit VPathBuilder(const BuildOptions& options) : options_(options) {}
  
  // ========================================================================
  // MAIN BUILD METHODS
  // ========================================================================
  
  /**
   * @brief Build VPath from RichPathResult
   */
  vrobot_local_planner::msg::Path buildFromRichPath(
    const data_structures::RichPathResult& rich_path) const;
  
  /**
   * @brief Build VPath from enhanced path segments
   */
  vrobot_local_planner::msg::Path buildFromEnhancedSegments(
    const std::vector<data_structures::EnhancedPathSegment>& segments) const;
  
  /**
   * @brief Build VPath from legacy path segments
   */
  vrobot_local_planner::msg::Path buildFromLegacySegments(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segments,
    const data_structures::PathAttributes& default_attrs = {}) const;
  
  // ========================================================================
  // CONFIGURATION
  // ========================================================================
  
  /**
   * @brief Set build options
   */
  VPathBuilder& setOptions(const BuildOptions& options) {
    options_ = options;
    return *this;
  }
  
  /**
   * @brief Set interpolation resolution
   */
  VPathBuilder& setResolution(double resolution) {
    options_.interpolation_resolution = resolution;
    return *this;
  }
  
  /**
   * @brief Set default velocity
   */
  VPathBuilder& setDefaultVelocity(double velocity) {
    options_.default_velocity = velocity;
    return *this;
  }
  
  /**
   * @brief Set frame ID and timestamp
   */
  VPathBuilder& setFrame(const std::string& frame_id, const rclcpp::Time& timestamp) {
    options_.frame_id = frame_id;
    options_.timestamp = timestamp;
    return *this;
  }
  
  /**
   * @brief Enable/disable velocity smoothing
   */
  VPathBuilder& setVelocitySmoothing(bool enable, double max_accel = 1.0) {
    options_.apply_velocity_smoothing = enable;
    options_.max_acceleration = max_accel;
    return *this;
  }
  
  /**
   * @brief Enable/disable curvature constraints
   */
  VPathBuilder& setCurvatureConstraints(bool enable, double min_velocity = 0.2) {
    options_.apply_curvature_constraints = enable;
    options_.min_curvature_velocity = min_velocity;
    return *this;
  }
  
  // ========================================================================
  // STATIC CONVENIENCE METHODS
  // ========================================================================
  
  /**
   * @brief Quick build with default options
   */
  static vrobot_local_planner::msg::Path quickBuild(
    const data_structures::RichPathResult& rich_path,
    const std::string& frame_id = "map",
    const rclcpp::Time& timestamp = rclcpp::Time(0),
    double resolution = 0.02);
  
  /**
   * @brief Build with custom velocity profile
   */
  static vrobot_local_planner::msg::Path buildWithVelocityProfile(
    const data_structures::RichPathResult& rich_path,
    const std::vector<double>& velocity_profile,
    const std::string& frame_id = "map",
    const rclcpp::Time& timestamp = rclcpp::Time(0),
    double resolution = 0.02);
  
  /**
   * @brief Build with uniform velocity
   */
  static vrobot_local_planner::msg::Path buildWithUniformVelocity(
    const data_structures::RichPathResult& rich_path,
    double velocity,
    const std::string& frame_id = "map",
    const rclcpp::Time& timestamp = rclcpp::Time(0),
    double resolution = 0.02);
  
private:
  // ========================================================================
  // INTERNAL METHODS
  // ========================================================================
  
  /**
   * @brief Apply velocity smoothing to a velocity profile
   */
  std::vector<double> applySmoothingToVelocities(
    const std::vector<double>& velocities,
    const std::vector<double>& distances) const;
  
  /**
   * @brief Apply curvature-based velocity constraints
   */
  std::vector<double> applyCurvatureConstraints(
    const std::vector<double>& velocities,
    const std::vector<Eigen::Vector3d>& poses) const;
  
  /**
   * @brief Convert pose and velocity to PlannerPose message
   */
  vrobot_local_planner::msg::PlannerPose createPlannerPose(
    const Eigen::Vector3d& pose, double velocity) const;
  
  /**
   * @brief Convert angle to quaternion
   */
  std::array<double, 4> angleToQuaternion(double theta) const;
  
  BuildOptions options_;
};

// ========================================================================
// SPECIALIZED BUILDERS
// ========================================================================

/**
 * @brief Builder for emergency/safety paths
 */
class EmergencyVPathBuilder : public VPathBuilder {
public:
  EmergencyVPathBuilder() {
    BuildOptions opts;
    opts.default_velocity = 0.3;  // Slower for safety
    opts.apply_velocity_smoothing = true;
    opts.max_acceleration = 0.5;  // Gentle acceleration
    opts.apply_curvature_constraints = true;
    opts.min_curvature_velocity = 0.1;  // Very slow around curves
    setOptions(opts);
  }
};

/**
 * @brief Builder for high-speed paths
 */
class HighSpeedVPathBuilder : public VPathBuilder {
public:
  HighSpeedVPathBuilder() {
    BuildOptions opts;
    opts.default_velocity = 2.0;  // Faster default
    opts.apply_velocity_smoothing = true;
    opts.max_acceleration = 2.0;  // More aggressive acceleration
    opts.apply_curvature_constraints = true;
    opts.min_curvature_velocity = 0.5;  // Still reduce speed in curves
    setOptions(opts);
  }
};

/**
 * @brief Builder for precision paths (narrow spaces, docking)
 */
class PrecisionVPathBuilder : public VPathBuilder {
public:
  PrecisionVPathBuilder() {
    BuildOptions opts;
    opts.interpolation_resolution = 0.01;  // Higher resolution
    opts.default_velocity = 0.2;  // Very slow for precision
    opts.apply_velocity_smoothing = true;
    opts.max_acceleration = 0.3;  // Very gentle
    opts.apply_curvature_constraints = true;
    opts.min_curvature_velocity = 0.05;  // Almost stop for curves
    setOptions(opts);
  }
};

} // namespace utils
} // namespace vrobot_route_follow