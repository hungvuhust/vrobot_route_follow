#include "vrobot_route_follow/utils/vpath_builder.hpp"
#include "vrobot_route_follow/data_structures/enhanced_path_segment.hpp"
#include <algorithm>
#include <cmath>

namespace vrobot_route_follow {
namespace utils {

// ========================================================================
// MAIN BUILD METHODS
// ========================================================================

vrobot_local_planner::msg::Path VPathBuilder::buildFromRichPath(
    const data_structures::RichPathResult& rich_path) const {
  
  vrobot_local_planner::msg::Path vpath;
  
  // Set header
  vpath.header.frame_id = options_.frame_id;
  vpath.header.stamp = options_.timestamp;
  
  if (!rich_path.success || rich_path.nodeSequence.empty()) {
    return vpath; // Return empty path
  }
  
  // Build enhanced segments from rich path
  std::vector<data_structures::EnhancedPathSegment> segments;
  segments.reserve(rich_path.nodeSequence.size() - 1);
  
  for (size_t i = 0; i < rich_path.nodeSequence.size() - 1; ++i) {
    const auto& start_node = rich_path.nodeSequence[i];
    const auto& end_node = rich_path.nodeSequence[i + 1];
    
    data_structures::EnhancedPathSegment segment;
    segment.start_pose = start_node.toPose();
    segment.end_pose = end_node.toPose();
    segment.segment_id = static_cast<int32_t>(i);
    segment.source_node_ids = {start_node.id, end_node.id};
    
    // Set attributes from link if available
    if (i < rich_path.linkSequence.size()) {
      const auto& link = rich_path.linkSequence[i];
      segment.source_link_id = link.id_straight_link;
      
      // Convert link attributes to path attributes
      data_structures::PathAttributes attrs;
      attrs.max_velocity = link.max_velocity;
      attrs.target_velocity = link.max_velocity;
      
      if (link.link_type.has_value()) {
        attrs.path_type = link.link_type.value();
      }
      
      if (link.width.has_value()) {
        attrs.path_width = link.width.value();
      }
      
      if (link.traffic_direction.has_value()) {
        attrs.traffic_direction = link.traffic_direction.value();
      }
      
      if (link.is_emergency_path.has_value()) {
        attrs.is_emergency_path = link.is_emergency_path.value();
      }
      
      segment.attributes = attrs;
    } else {
      // Use default attributes
      segment.attributes.target_velocity = options_.default_velocity;
    }
    
    segments.push_back(std::move(segment));
  }
  
  return buildFromEnhancedSegments(segments);
}

vrobot_local_planner::msg::Path VPathBuilder::buildFromEnhancedSegments(
    const std::vector<data_structures::EnhancedPathSegment>& segments) const {
  
  vrobot_local_planner::msg::Path vpath;
  
  // Set header
  vpath.header.frame_id = options_.frame_id;
  vpath.header.stamp = options_.timestamp;
  
  if (segments.empty()) {
    return vpath;
  }
  
  // Apply processing
  auto processed_segments = segments;
  
  // Apply curvature constraints
  if (options_.apply_curvature_constraints) {
    for (auto& segment : processed_segments) {
      double curvature = segment.getCurvature();
      
      if (curvature > 0.1) { // High curvature threshold
        double curvature_velocity = std::max(
          options_.min_curvature_velocity,
          segment.attributes.getEffectiveVelocity() * 0.5);
        
        if (!segment.attributes.target_velocity.has_value() ||
            segment.attributes.target_velocity.value() > curvature_velocity) {
          segment.attributes.target_velocity = curvature_velocity;
        }
      }
    }
  }
  
  // Apply velocity smoothing
  if (options_.apply_velocity_smoothing) {
    processed_segments = data_structures::enhanced_path_utils::applySmoothVelocity(
      processed_segments, options_.max_acceleration);
  }
  
  // Generate VPath poses
  auto vpath_poses = data_structures::enhanced_path_utils::generateVPath(
    processed_segments, options_.interpolation_resolution);
  
  // Convert to VPath message
  vpath.poses.reserve(vpath_poses.size());
  for (const auto& [pose, velocity] : vpath_poses) {
    vpath.poses.push_back(createPlannerPose(pose, velocity));
  }
  
  return vpath;
}

vrobot_local_planner::msg::Path VPathBuilder::buildFromLegacySegments(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segments,
    const data_structures::PathAttributes& default_attrs) const {
  
  // Convert to enhanced segments
  auto enhanced_segments = data_structures::enhanced_path_utils::fromLegacySegments(
    segments, default_attrs);
  
  // Use enhanced segment builder
  return buildFromEnhancedSegments(enhanced_segments);
}

// ========================================================================
// STATIC CONVENIENCE METHODS
// ========================================================================

vrobot_local_planner::msg::Path VPathBuilder::quickBuild(
    const data_structures::RichPathResult& rich_path,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double resolution) {
  
  BuildOptions options;
  options.frame_id = frame_id;
  options.timestamp = timestamp;
  options.interpolation_resolution = resolution;
  
  VPathBuilder builder(options);
  return builder.buildFromRichPath(rich_path);
}

vrobot_local_planner::msg::Path VPathBuilder::buildWithVelocityProfile(
    const data_structures::RichPathResult& rich_path,
    const std::vector<double>& velocity_profile,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double resolution) {
  
  BuildOptions options;
  options.frame_id = frame_id;
  options.timestamp = timestamp;
  options.interpolation_resolution = resolution;
  options.apply_velocity_smoothing = false; // Don't smooth custom profile
  
  VPathBuilder builder(options);
  
  // Convert rich path to enhanced segments and apply custom velocity profile
  auto enhanced_segments = data_structures::enhanced_path_utils::fromLegacySegments(
    rich_path.toPathSegments(), data_structures::PathAttributes{});
  
  // Apply custom velocity profile
  for (size_t i = 0; i < enhanced_segments.size() && i < velocity_profile.size(); ++i) {
    enhanced_segments[i].attributes.target_velocity = velocity_profile[i];
  }
  
  return builder.buildFromEnhancedSegments(enhanced_segments);
}

vrobot_local_planner::msg::Path VPathBuilder::buildWithUniformVelocity(
    const data_structures::RichPathResult& rich_path,
    double velocity,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double resolution) {
  
  BuildOptions options;
  options.frame_id = frame_id;
  options.timestamp = timestamp;
  options.interpolation_resolution = resolution;
  options.default_velocity = velocity;
  options.apply_velocity_smoothing = false; // Use uniform velocity
  options.apply_curvature_constraints = false;
  
  VPathBuilder builder(options);
  return builder.buildFromRichPath(rich_path);
}

// ========================================================================
// INTERNAL METHODS
// ========================================================================

vrobot_local_planner::msg::PlannerPose VPathBuilder::createPlannerPose(
    const Eigen::Vector3d& pose, double velocity) const {
  
  vrobot_local_planner::msg::PlannerPose planner_pose;
  
  // Header
  planner_pose.header.frame_id = options_.frame_id;
  planner_pose.header.stamp = options_.timestamp;
  
  // Position
  planner_pose.pose.position.x = pose(0);
  planner_pose.pose.position.y = pose(1);
  planner_pose.pose.position.z = 0.0;
  
  // Orientation from theta
  auto quat = angleToQuaternion(pose(2));
  planner_pose.pose.orientation.x = quat[0];
  planner_pose.pose.orientation.y = quat[1];
  planner_pose.pose.orientation.z = quat[2];
  planner_pose.pose.orientation.w = quat[3];
  
  // Velocity
  planner_pose.speed = static_cast<float>(velocity);
  
  return planner_pose;
}

std::array<double, 4> VPathBuilder::angleToQuaternion(double theta) const {
  double half_theta = theta * 0.5;
  double cos_half = std::cos(half_theta);
  double sin_half = std::sin(half_theta);
  
  // Quaternion for rotation around Z-axis: [x, y, z, w]
  return {0.0, 0.0, sin_half, cos_half};
}

} // namespace utils
} // namespace vrobot_route_follow