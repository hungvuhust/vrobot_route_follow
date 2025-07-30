#pragma once

#include "../data_structures/rich_path_result.hpp"
#include "../data_structures/node_info.hpp" 
#include "../data_structures/link_info.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <unordered_map>

namespace vrobot_route_follow {
namespace utils {



// ========================================================================
// MODULAR ARCHITECTURE VISUALIZATION
// ========================================================================

/**
 * @brief Rich visualization utilities for the new modular architecture
 */
class RichVisualization {
public:
  /**
   * @brief Create visualization markers for RichPathResult
   * @param rich_result RichPathResult to visualize
   * @param frame_id Frame ID for markers
   * @param timestamp Timestamp for markers
   * @param show_nodes Whether to show node markers
   * @param show_links Whether to show link markers
   * @param show_poses Whether to show pose sequence
   * @param show_velocity Whether to show velocity profile
   * @return ROS2 MarkerArray message
   */
  static visualization_msgs::msg::MarkerArray createRichPathMarkers(
      const vrobot_route_follow::data_structures::RichPathResult& rich_result,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      bool show_nodes = true,
      bool show_links = true, 
      bool show_poses = true,
      bool show_velocity = false);

  /**
   * @brief Create visualization markers for map data
   * @param nodes Map nodes to visualize
   * @param links Map links to visualize  
   * @param frame_id Frame ID for markers
   * @param timestamp Timestamp for markers
   * @param node_scale Scale for node markers
   * @param link_scale Scale for link markers
   * @return ROS2 MarkerArray message
   */
  static visualization_msgs::msg::MarkerArray createMapMarkers(
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      double node_scale = 0.1,
      double link_scale = 0.05);

  /**
   * @brief Create color message
   * @param r Red component (0-1)
   * @param g Green component (0-1)
   * @param b Blue component (0-1)
   * @param a Alpha component (0-1)
   * @return Color message
   */
  static std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a = 1.0);

  /**
   * @brief Clear all visualization markers
   * @param frame_id Frame ID for markers
   * @param timestamp Timestamp for markers
   * @return Clear markers array
   */
  static visualization_msgs::msg::MarkerArray createClearMarkers(
      const std::string& frame_id,
      const rclcpp::Time& timestamp);
};

} // namespace utils
} // namespace vrobot_route_follow