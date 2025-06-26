#include "vrobot_route_follow/utils/nav_conversion.hpp"

namespace vrobot_route_follow {
namespace utils {

// Explicit template instantiations for common types
template class NavConversion<int, CPose2D, double>;
template class NavConversion<long, CPose2D, double>;
template class NavConversion<unsigned long, CPose2D, double>;

template <typename NodeID, typename Pose2D, typename WeightType>
std::vector<Pose2D> NavConversion<NodeID, Pose2D, WeightType>::interpolatePoses(
    const std::vector<PathSegment> &pathSegments, double resolution) const {

  std::vector<Pose2D> interpolatedPoses;

  if (pathSegments.empty()) {
    return interpolatedPoses;
  }

  // Add starting pose
  interpolatedPoses.push_back(pathSegments[0].first);

  for (const auto &segment : pathSegments) {
    const Pose2D &start = segment.first;
    const Pose2D &end   = segment.second;

    double dx            = end.x() - start.x();
    double dy            = end.y() - start.y();
    double segmentLength = std::sqrt(dx * dx + dy * dy);

    if (segmentLength < resolution) {
      // Segment too short, just add end point
      interpolatedPoses.push_back(end);
      continue;
    }

    int numSteps = static_cast<int>(std::ceil(segmentLength / resolution));

    for (int i = 1; i <= numSteps; ++i) {
      double t       = static_cast<double>(i) / numSteps;
      double interpX = start.x() + t * dx;
      double interpY = start.y() + t * dy;

      // Point-to-point orientation (direction of movement)
      double theta = std::atan2(dy, dx);

      interpolatedPoses.emplace_back(interpX, interpY, theta);
    }
  }

  return interpolatedPoses;
}

template <typename NodeID, typename Pose2D, typename WeightType>
nav_msgs::msg::Path NavConversion<NodeID, Pose2D, WeightType>::toNavPath(
    const std::vector<PathSegment> &pathSegments, const std::string &frameId,
    const rclcpp::Time &timestamp, double resolution) const {

  nav_msgs::msg::Path navPath;
  navPath.header.frame_id = frameId;
  navPath.header.stamp    = timestamp;

  if (pathSegments.empty()) {
    return navPath;
  }

  // Interpolate poses
  std::vector<Pose2D> interpolatedPoses =
      interpolatePoses(pathSegments, resolution);

  // Convert to ROS2 poses
  navPath.poses.reserve(interpolatedPoses.size());

  for (const auto &pose : interpolatedPoses) {
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.frame_id = frameId;
    poseStamped.header.stamp    = timestamp;

    poseStamped.pose.position.x = pose.x();
    poseStamped.pose.position.y = pose.y();
    poseStamped.pose.position.z = 0.0;

    // Convert angle to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.phi());
    poseStamped.pose.orientation = tf2::toMsg(q);

    navPath.poses.push_back(poseStamped);
  }

  return navPath;
}

// Explicit instantiations for the template methods
template std::vector<CPose2D>
NavConversion<int, CPose2D, double>::interpolatePoses(
    const std::vector<PathSegment> &, double) const;

template nav_msgs::msg::Path NavConversion<int, CPose2D, double>::toNavPath(
    const std::vector<PathSegment> &, const std::string &, const rclcpp::Time &,
    double) const;

template std::vector<CPose2D>
NavConversion<unsigned long, CPose2D, double>::interpolatePoses(
    const std::vector<PathSegment> &, double) const;

template nav_msgs::msg::Path
NavConversion<unsigned long, CPose2D, double>::toNavPath(
    const std::vector<PathSegment> &, const std::string &, const rclcpp::Time &,
    double) const;

} // namespace utils
} // namespace vrobot_route_follow