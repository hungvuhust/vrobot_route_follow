#pragma once

#include "../core/graph_base.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mrpt_graphPose_pose {
namespace utils {

/**
 * @brief Visualization utilities for ROS2 MarkerArray and MRPT
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class Visualization
    : public virtual core::GraphBase<NodeID, Pose2D, WeightType> {
public:
  using Base        = core::GraphBase<NodeID, Pose2D, WeightType>;
  using PathSegment = std::pair<Pose2D, Pose2D>;

  // ========================================================================
  // ROS2 MARKER ARRAY VISUALIZATION
  // ========================================================================

  /**
   * @brief Convert graph to ROS2 MarkerArray for visualization
   * @param frameId Frame ID for markers
   * @param timestamp Timestamp for markers
   * @param nodeScale Scale factor for node markers
   * @param edgeScale Scale factor for edge markers
   * @param showNodeLabels Whether to show node ID labels
   * @param showLinkDirections Whether to show link direction arrows
   * @return ROS2 MarkerArray message
   */
  visualization_msgs::msg::MarkerArray
  toMarkerArray(const std::string &frameId, const rclcpp::Time &timestamp,
                double nodeScale = 0.1, double edgeScale = 0.05,
                bool showNodeLabels     = true,
                bool showLinkDirections = true) const;

  /**
   * @brief Create path visualization markers
   * @param pathSegments Path segments to visualize
   * @param frameId Frame ID for markers
   * @param timestamp Timestamp for markers
   * @param pathColor Color for path visualization
   * @param lineWidth Width of path lines
   * @return ROS2 MarkerArray for path
   */
  visualization_msgs::msg::MarkerArray createPathMarkers(
      const std::vector<PathSegment> &pathSegments, const std::string &frameId,
      const rclcpp::Time             &timestamp,
      const std_msgs::msg::ColorRGBA &pathColor = createColor(0.0, 0.0, 1.0,
                                                              1.0),
      double                          lineWidth = 0.1) const;

  /**
   * @brief Create multi-colored path visualization for different segments
   * @param coloredSegments Path segments with types and colors
   * @param frameId Frame ID for markers
   * @param timestamp Timestamp for markers
   * @param lineWidth Width of path lines
   * @return ROS2 MarkerArray for colored path
   */
  visualization_msgs::msg::MarkerArray createColoredPathMarkers(
      const std::vector<std::tuple<PathSegment, std::string,
                                   std_msgs::msg::ColorRGBA>> &coloredSegments,
      const std::string &frameId, const rclcpp::Time &timestamp,
      double lineWidth = 0.1) const;

  // ========================================================================
  // UTILITY FUNCTIONS
  // ========================================================================

  /**
   * @brief Create color message
   * @param r Red component (0-1)
   * @param g Green component (0-1)
   * @param b Blue component (0-1)
   * @param a Alpha component (0-1)
   * @return Color message
   */
  static std_msgs::msg::ColorRGBA createColor(double r, double g, double b,
                                              double a = 1.0);

  /**
   * @brief Get predefined colors for different path segments
   * @return Map of segment types to colors
   */
  static std::map<std::string, std_msgs::msg::ColorRGBA> getSegmentColors();

  /**
   * @brief Clear all visualization markers
   * @param frameId Frame ID for markers
   * @param timestamp Timestamp for markers
   * @return MarkerArray with DELETE_ALL action
   */
  static visualization_msgs::msg::MarkerArray
  createClearMarkers(const std::string &frameId, const rclcpp::Time &timestamp);

  // ========================================================================
  // ANALYSIS VISUALIZATION
  // ========================================================================

  /**
   * @brief Create markers for link analysis visualization
   * @param queryPose Query pose for analysis
   * @param closestLinks Links to visualize
   * @param frameId Frame ID for markers
   * @param timestamp Timestamp for markers
   * @return MarkerArray for link analysis
   */
  visualization_msgs::msg::MarkerArray createLinkAnalysisMarkers(
      const Pose2D                                          &queryPose,
      const std::vector<std::tuple<NodeID, NodeID, double>> &closestLinks,
      const std::string &frameId, const rclcpp::Time &timestamp) const;
};

} // namespace utils
} // namespace mrpt_graphPose_pose