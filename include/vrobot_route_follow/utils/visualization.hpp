#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/time.hpp>

#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/curve_link_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"

#include <string>
#include <unordered_map>

namespace vrobot_route_follow {
namespace utils {

/**
 * @brief Rich visualization utilities for the new modular architecture
 * 
 * Provides visualization methods for RichPathResult, map data, and curved paths
 * with support for Bézier curves and enhanced path segments.
 */
class RichVisualization {
public:
  /**
   * @brief Create visualization markers for a rich path result
   * @param rich_result The rich path result to visualize
   * @param frame_id Frame ID for the markers
   * @param timestamp Timestamp for the markers
   * @param node_scale Scale for node markers
   * @param link_scale Scale for link markers
   * @return MarkerArray containing path visualization markers
   */
  static visualization_msgs::msg::MarkerArray createRichPathMarkers(
      const vrobot_route_follow::data_structures::RichPathResult& rich_result,
      const std::string& frame_id = "map",
      const rclcpp::Time& timestamp = rclcpp::Time(0),
      double node_scale = 0.15,
      double link_scale = 0.05);

  /**
   * @brief Create visualization markers for map data (nodes, links, and curved links)
   * @param nodes Map of node IDs to NodeInfo
   * @param links Map of link IDs to LinkInfo
   * @param curved_links Map of curved link IDs to CurveLinkInfo
   * @param frame_id Frame ID for the markers
   * @param timestamp Timestamp for the markers
   * @param node_scale Scale for node markers
   * @param link_scale Scale for link markers
   * @return MarkerArray containing all map visualization markers
   */
  static visualization_msgs::msg::MarkerArray createMapMarkers(
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::CurveLinkInfo>& curved_links,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      double node_scale = 0.1,
      double link_scale = 0.05);

  /**
   * @brief Create color from RGBA values
   * @param r Red component (0-1)
   * @param g Green component (0-1)
   * @param b Blue component (0-1)
   * @param a Alpha component (0-1)
   * @return ColorRGBA message
   */
  static std_msgs::msg::ColorRGBA createColor(double r, double g, double b, double a = 1.0);

  /**
   * @brief Create markers to clear all visualizations
   * @param frame_id Frame ID for the markers
   * @param timestamp Timestamp for the markers
   * @return MarkerArray containing delete markers
   */
  static visualization_msgs::msg::MarkerArray createClearMarkers(
      const std::string& frame_id = "map",
      const rclcpp::Time& timestamp = rclcpp::Time(0));

private:
  // Utility functions for marker creation
  static void addNodesMarkers(
      visualization_msgs::msg::MarkerArray& markers,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      double scale);

  static void addLinksMarkers(
      visualization_msgs::msg::MarkerArray& markers,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      double scale);

  static void addCurvedLinksMarkers(
      visualization_msgs::msg::MarkerArray& markers,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::CurveLinkInfo>& curved_links,
      const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
      const std::string& frame_id,
      const rclcpp::Time& timestamp,
      double scale);
};

} // namespace utils
} // namespace vrobot_route_follow