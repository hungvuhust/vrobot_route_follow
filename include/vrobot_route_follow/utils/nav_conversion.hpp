#pragma once

#include "../core/graph_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vrobot_route_follow {
namespace utils {

/**
 * @brief Navigation conversion utilities for ROS2 integration
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class NavConversion
    : public virtual core::GraphBase<NodeID, Pose2D, WeightType> {
public:
  using Base        = core::GraphBase<NodeID, Pose2D, WeightType>;
  using PathSegment = std::pair<Pose2D, Pose2D>;

  // ========================================================================
  // PATH INTERPOLATION
  // ========================================================================

  /**
   * @brief Interpolate poses along path segments with configurable resolution
   * @param pathSegments Vector of path segments
   * @param resolution Distance between interpolated poses (meters)
   * @return Vector of interpolated poses
   */
  std::vector<Pose2D>
  interpolatePoses(const std::vector<PathSegment> &pathSegments,
                   double                          resolution = 0.01) const;

  // ========================================================================
  // ROS2 NAV_MSGS CONVERSION
  // ========================================================================

  /**
   * @brief Convert path segments to nav_msgs::msg::Path with interpolation
   * @param pathSegments Vector of path segments
   * @param frameId Frame ID for the path
   * @param timestamp Timestamp for the path
   * @param resolution Interpolation resolution (meters)
   * @return ROS2 nav_msgs::msg::Path
   */
  nav_msgs::msg::Path toNavPath(const std::vector<PathSegment> &pathSegments,
                                const std::string              &frameId,
                                const rclcpp::Time             &timestamp,
                                double resolution = 0.01) const;
};

} // namespace utils
} // namespace vrobot_route_follow