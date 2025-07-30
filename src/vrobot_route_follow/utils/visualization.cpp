#include "vrobot_route_follow/utils/visualization.hpp"
#include <iostream>

namespace vrobot_route_follow {
namespace utils {

















// ========================================================================
// NEW MODULAR ARCHITECTURE VISUALIZATION IMPLEMENTATION
// ========================================================================

visualization_msgs::msg::MarkerArray
RichVisualization::createRichPathMarkers(
    const vrobot_route_follow::data_structures::RichPathResult& rich_result,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    bool show_nodes,
    bool show_links, 
    bool show_poses,
    bool show_velocity) {

  visualization_msgs::msg::MarkerArray markers;

  // 1. Visualize planned nodes sequence
  if (show_nodes && !rich_result.nodeSequence.empty()) {
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = frame_id;
    node_marker.header.stamp = timestamp;
    node_marker.ns = "rich_path_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = 0.15;
    node_marker.scale.y = 0.15;
    node_marker.scale.z = 0.15;
    node_marker.color = createColor(1.0, 0.5, 0.0, 1.0); // Orange

    for (const auto& node : rich_result.nodeSequence) {
      geometry_msgs::msg::Point point;
      point.x = node.x;
      point.y = node.y;
      point.z = 0.0;
      node_marker.points.push_back(point);
    }
    markers.markers.push_back(node_marker);
  }

  // 2. Visualize links sequence
  if (show_links && !rich_result.linkSequence.empty()) {
    visualization_msgs::msg::Marker link_marker;
    link_marker.header.frame_id = frame_id;
    link_marker.header.stamp = timestamp;
    link_marker.ns = "rich_path_links";
    link_marker.id = 1;
    link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    link_marker.action = visualization_msgs::msg::Marker::ADD;
    link_marker.scale.x = 0.08;
    link_marker.color = createColor(0.0, 1.0, 1.0, 0.8); // Cyan

    // Note: LinkInfo only has node IDs (id_start, id_end), not positions
    // For link visualization, we'll use the node sequence to draw connections
    for (size_t i = 0; i < rich_result.nodeSequence.size() - 1; ++i) {
      const auto& start_node = rich_result.nodeSequence[i];
      const auto& end_node = rich_result.nodeSequence[i + 1];
      
      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = start_node.x;
      start_point.y = start_node.y;
      start_point.z = 0.0;
      
      end_point.x = end_node.x;
      end_point.y = end_node.y;
      end_point.z = 0.0;

      link_marker.points.push_back(start_point);
      link_marker.points.push_back(end_point);
    }
    markers.markers.push_back(link_marker);
  }

  // 3. Visualize pose sequence (interpolated path)
  if (show_poses && !rich_result.poseSequence.empty()) {
    visualization_msgs::msg::Marker pose_marker;
    pose_marker.header.frame_id = frame_id;
    pose_marker.header.stamp = timestamp;
    pose_marker.ns = "rich_path_poses";
    pose_marker.id = 2;
    pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.scale.x = 0.05;
    pose_marker.color = createColor(1.0, 0.0, 1.0, 1.0); // Magenta

    for (const auto& pose : rich_result.poseSequence) {
      geometry_msgs::msg::Point point;
      point.x = pose.x();
      point.y = pose.y();
      point.z = 0.0;
      pose_marker.points.push_back(point);
    }
    markers.markers.push_back(pose_marker);
  }

  // 4. Visualize velocity profile (optional)
  if (show_velocity && !rich_result.velocityProfile.empty() && 
      rich_result.velocityProfile.size() == rich_result.poseSequence.size()) {
    
    for (size_t i = 0; i < rich_result.poseSequence.size(); ++i) {
      visualization_msgs::msg::Marker vel_marker;
      vel_marker.header.frame_id = frame_id;
      vel_marker.header.stamp = timestamp;
      vel_marker.ns = "velocity_profile";
      vel_marker.id = static_cast<int>(i + 100);
      vel_marker.type = visualization_msgs::msg::Marker::ARROW;
      vel_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Scale arrow by velocity
      double velocity = rich_result.velocityProfile[i];
      vel_marker.scale.x = velocity * 0.1; // Length
      vel_marker.scale.y = 0.02; // Width  
      vel_marker.scale.z = 0.02; // Height
      
      // Color by velocity (green = fast, red = slow)
      double normalized_vel = std::min(velocity / 2.0, 1.0);
      vel_marker.color = createColor(1.0 - normalized_vel, normalized_vel, 0.0, 0.7);
      
      vel_marker.pose.position.x = rich_result.poseSequence[i].x();
      vel_marker.pose.position.y = rich_result.poseSequence[i].y();
      vel_marker.pose.position.z = 0.0;
      
      // Set orientation from pose theta
      tf2::Quaternion quat;
      quat.setRPY(0, 0, rich_result.poseSequence[i].z());
      vel_marker.pose.orientation.x = quat.x();
      vel_marker.pose.orientation.y = quat.y();
      vel_marker.pose.orientation.z = quat.z();
      vel_marker.pose.orientation.w = quat.w();
      
      markers.markers.push_back(vel_marker);
    }
  }

  return markers;
}

visualization_msgs::msg::MarkerArray
RichVisualization::createMapMarkers(
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double node_scale,
    double link_scale) {

  visualization_msgs::msg::MarkerArray markers;

  // 1. Map nodes
  if (!nodes.empty()) {
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = frame_id;
    node_marker.header.stamp = timestamp;
    node_marker.ns = "map_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = node_scale;
    node_marker.scale.y = node_scale;
    node_marker.scale.z = node_scale;
    node_marker.color = createColor(0.0, 1.0, 0.0, 0.8); // Green

    for (const auto& [node_id, node_info] : nodes) {
      geometry_msgs::msg::Point point;
      point.x = node_info.x;
      point.y = node_info.y;
      point.z = 0.0;
      node_marker.points.push_back(point);
    }
    markers.markers.push_back(node_marker);
  }

  // 2. Map links
  if (!links.empty()) {
    visualization_msgs::msg::Marker link_marker;
    link_marker.header.frame_id = frame_id;
    link_marker.header.stamp = timestamp;
    link_marker.ns = "map_links";
    link_marker.id = 1;
    link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    link_marker.action = visualization_msgs::msg::Marker::ADD;
    link_marker.scale.x = link_scale;
    link_marker.color = createColor(0.5, 0.5, 0.5, 0.6); // Gray

    for (const auto& [link_id, link_info] : links) {
      // Find start and end nodes from the nodes map
      auto start_it = nodes.find(link_info.id_start);
      auto end_it = nodes.find(link_info.id_end);
      
      if (start_it != nodes.end() && end_it != nodes.end()) {
        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = start_it->second.x;
        start_point.y = start_it->second.y;
        start_point.z = 0.0;
        
        end_point.x = end_it->second.x;
        end_point.y = end_it->second.y;
        end_point.z = 0.0;

        link_marker.points.push_back(start_point);
        link_marker.points.push_back(end_point);
      }
    }
    markers.markers.push_back(link_marker);
  }

  return markers;
}

std_msgs::msg::ColorRGBA
RichVisualization::createColor(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

visualization_msgs::msg::MarkerArray
RichVisualization::createClearMarkers(
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
  
  visualization_msgs::msg::MarkerArray clear_markers;
  
  // Clear all namespaces used in rich visualization
  std::vector<std::string> namespaces = {
    "rich_path_nodes", "rich_path_links", "rich_path_poses", 
    "velocity_profile", "map_nodes", "map_links"
  };
  
  for (const auto& ns : namespaces) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = timestamp;
    clear_marker.ns = ns;
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear_marker);
  }
  
  return clear_markers;
}

} // namespace utils
} // namespace vrobot_route_follow