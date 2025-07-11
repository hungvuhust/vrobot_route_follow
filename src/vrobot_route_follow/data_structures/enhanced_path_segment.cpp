#include "vrobot_route_follow/data_structures/enhanced_path_segment.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace vrobot_route_follow {
namespace data_structures {

// ========================================================================
// ENHANCED PATH UTILS IMPLEMENTATION
// ========================================================================

namespace enhanced_path_utils {

EnhancedPathSegments fromLegacySegments(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                         &legacy_segments,
    const PathAttributes &default_attributes) {

  EnhancedPathSegments enhanced_segments;
  enhanced_segments.reserve(legacy_segments.size());

  for (size_t i = 0; i < legacy_segments.size(); ++i) {
    const auto &segment = legacy_segments[i];

    EnhancedPathSegment enhanced;
    enhanced.start_pose = segment.first;
    enhanced.end_pose   = segment.second;
    enhanced.attributes = default_attributes;
    enhanced.segment_id = static_cast<int32_t>(i);

    enhanced_segments.push_back(std::move(enhanced));
  }

  return enhanced_segments;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
toLegacySegments(const EnhancedPathSegments &enhanced_segments) {

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> legacy_segments;
  legacy_segments.reserve(enhanced_segments.size());

  for (const auto &segment : enhanced_segments) {
    legacy_segments.emplace_back(segment.start_pose, segment.end_pose);
  }

  return legacy_segments;
}

double calculateTotalLength(const EnhancedPathSegments &segments) {
  double total_length = 0.0;

  for (const auto &segment : segments) {
    total_length += segment.getLength();
  }

  return total_length;
}

double calculateTotalTime(const EnhancedPathSegments &segments) {
  double total_time = 0.0;

  for (const auto &segment : segments) {
    double length   = segment.getLength();
    double velocity = segment.attributes.getEffectiveVelocity();

    if (velocity > 0.0) {
      total_time += length / velocity;
    }
  }

  return total_time;
}

EnhancedPathSegments applySmoothVelocity(const EnhancedPathSegments &segments,
                                         double max_acceleration) {

  if (segments.empty()) {
    return segments;
  }

  EnhancedPathSegments smoothed_segments = segments;

  // Forward pass: limit acceleration
  for (size_t i = 1; i < smoothed_segments.size(); ++i) {
    auto &prev_segment = smoothed_segments[i - 1];
    auto &curr_segment = smoothed_segments[i];

    double prev_velocity  = prev_segment.attributes.getEffectiveVelocity();
    double curr_velocity  = curr_segment.attributes.getEffectiveVelocity();
    double segment_length = prev_segment.getLength();

    if (segment_length > 0.0) {
      // Calculate maximum achievable velocity given acceleration constraint
      double max_achievable_velocity =
          std::sqrt(prev_velocity * prev_velocity +
                    2 * max_acceleration * segment_length);

      if (curr_velocity > max_achievable_velocity) {
        curr_segment.attributes.target_velocity = max_achievable_velocity;
      }
    }
  }

  // Backward pass: limit deceleration
  for (int i = static_cast<int>(smoothed_segments.size()) - 2; i >= 0; --i) {
    auto &curr_segment = smoothed_segments[i];
    auto &next_segment = smoothed_segments[i + 1];

    double curr_velocity  = curr_segment.attributes.getEffectiveVelocity();
    double next_velocity  = next_segment.attributes.getEffectiveVelocity();
    double segment_length = curr_segment.getLength();

    if (segment_length > 0.0) {
      // Calculate maximum velocity that allows deceleration to next segment
      double max_velocity_for_decel =
          std::sqrt(next_velocity * next_velocity +
                    2 * max_acceleration * segment_length);

      if (curr_velocity > max_velocity_for_decel) {
        curr_segment.attributes.target_velocity = max_velocity_for_decel;
      }
    }
  }

  return smoothed_segments;
}

std::vector<std::pair<Eigen::Vector3d, double>>
generateVPath(const EnhancedPathSegments &segments, double resolution) {

  std::vector<std::pair<Eigen::Vector3d, double>> vpath;

  for (const auto &segment : segments) {
    auto segment_vpath = segment.toVPathPoses(resolution);

    // Add all poses except the first one (to avoid duplication)
    if (vpath.empty()) {
      // For the first segment, add all poses
      vpath.insert(vpath.end(), segment_vpath.begin(), segment_vpath.end());
    } else {
      // For subsequent segments, skip the first pose
      if (!segment_vpath.empty()) {
        vpath.insert(vpath.end(), segment_vpath.begin() + 1,
                     segment_vpath.end());
      }
    }
  }

  return vpath;
}

} // namespace enhanced_path_utils

} // namespace data_structures
} // namespace vrobot_route_follow