#include "vrobot_route_follow/data_structures/rich_path_result.hpp"
#include "vrobot_route_follow/data_structures/enhanced_path_segment.hpp"
#include <limits>
#include <sstream>
#include <stdexcept>

namespace vrobot_route_follow {
namespace data_structures {

// Note: ROS message conversion methods are implemented in
// database_converter.cpp to avoid circular dependencies with ROS message
// headers

// ========================================================================
// SERIALIZATION METHODS
// ========================================================================

std::string RichPathResult::toJsonString() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"success\":" << (success ? "true" : "false") << ",";
  oss << "\"algorithm_used\":\"" << algorithmUsed << "\",";
  oss << "\"total_distance\":" << totalDistance;

  if (!errorMessage.empty()) {
    oss << ",\"error_message\":\"" << errorMessage << "\"";
  }

  if (totalTime.has_value()) {
    oss << ",\"total_time\":" << totalTime.value();
  }

  if (maxVelocity.has_value()) {
    oss << ",\"max_velocity\":" << maxVelocity.value();
  }

  if (minVelocity.has_value()) {
    oss << ",\"min_velocity\":" << minVelocity.value();
  }

  if (planningTimeMs.has_value()) {
    oss << ",\"planning_time_ms\":" << planningTimeMs.value();
  }

  // Node sequence
  oss << ",\"node_sequence\":[";
  for (size_t i = 0; i < nodeSequence.size(); ++i) {
    if (i > 0)
      oss << ",";
    oss << nodeSequence[i].toJsonString();
  }
  oss << "]";

  // Link sequence
  oss << ",\"link_sequence\":[";
  for (size_t i = 0; i < linkSequence.size(); ++i) {
    if (i > 0)
      oss << ",";
    oss << linkSequence[i].toJsonString();
  }
  oss << "]";

  // Pose sequence
  oss << ",\"pose_sequence\":[";
  for (size_t i = 0; i < poseSequence.size(); ++i) {
    if (i > 0)
      oss << ",";
    oss << "[" << poseSequence[i](0) << "," << poseSequence[i](1) << ","
        << poseSequence[i](2) << "]";
  }
  oss << "]";

  // Velocity profile
  oss << ",\"velocity_profile\":[";
  for (size_t i = 0; i < velocityProfile.size(); ++i) {
    if (i > 0)
      oss << ",";
    oss << velocityProfile[i];
  }
  oss << "]";

  // Metadata
  oss << ",\"metadata\":{";
  bool first = true;
  for (const auto &[key, value] : metadata) {
    if (!first)
      oss << ",";
    oss << "\"" << key << "\":" << value;
    first = false;
  }
  oss << "}";

  oss << "}";
  return oss.str();
}

RichPathResult RichPathResult::fromJsonString(const std::string &json_str) {
  // Simple JSON parsing - in production, should use proper JSON library
  RichPathResult result;

  // This is a simplified implementation
  // In a real implementation, you would use a proper JSON parser like
  // nlohmann/json

  // For now, throw an exception to indicate this needs proper implementation
  throw std::runtime_error(
      "RichPathResult::fromJsonString not fully implemented - needs proper "
      "JSON parser");

  return result;
}

// ========================================================================
// CONVERSION TO ENHANCED SEGMENTS
// ========================================================================

std::vector<EnhancedPathSegment> RichPathResult::toEnhancedSegments() const {
  std::vector<EnhancedPathSegment> segments;
  
  if (!success || nodeSequence.size() < 2) {
    return segments;
  }
  
  segments.reserve(nodeSequence.size() - 1);
  
  for (size_t i = 0; i < nodeSequence.size() - 1; ++i) {
    const auto& start_node = nodeSequence[i];
    const auto& end_node = nodeSequence[i + 1];
    
    EnhancedPathSegment segment;
    segment.start_pose = start_node.toPose();
    segment.end_pose = end_node.toPose();
    segment.segment_id = static_cast<int32_t>(i);
    segment.source_node_ids = {start_node.id, end_node.id};
    
    // Set attributes from link if available
    if (i < linkSequence.size()) {
      const auto& link = linkSequence[i];
      segment.source_link_id = link.id_straight_link;
      
      // Convert link attributes to path attributes
      PathAttributes attrs;
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
    }
    
    segments.push_back(std::move(segment));
  }
  
  return segments;
}

} // namespace data_structures
} // namespace vrobot_route_follow