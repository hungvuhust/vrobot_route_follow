#include "vrobot_route_follow/utils/visualization.hpp"

#include <rclcpp/rclcpp.hpp>

namespace vrobot_route_follow {
namespace utils {

// ========================================================================
// PUBLIC STATIC METHODS
// ========================================================================

visualization_msgs::msg::MarkerArray RichVisualization::createRichPathMarkers(
    const vrobot_route_follow::data_structures::RichPathResult& rich_result,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double node_scale,
    double link_scale) {

  visualization_msgs::msg::MarkerArray markers;
  
  if (!rich_result.success || rich_result.nodeSequence.empty()) {
    return markers; // Return empty array for failed paths
  }

  // 1. Path nodes
  if (!rich_result.nodeSequence.empty()) {
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = frame_id;
    node_marker.header.stamp = timestamp;
    node_marker.ns = "path_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = node_scale;
    node_marker.scale.y = node_scale;
    node_marker.scale.z = node_scale;
    node_marker.color = createColor(0.0, 1.0, 0.0, 0.8); // Green
    node_marker.pose.orientation.w = 1.0;

    for (const auto& node : rich_result.nodeSequence) {
      geometry_msgs::msg::Point point;
      point.x = node.x;
      point.y = node.y;
      point.z = 0.0;
      node_marker.points.push_back(point);
    }
    markers.markers.push_back(node_marker);
  }

  // 2. Path links (straight connections)
  if (rich_result.nodeSequence.size() > 1) {
    visualization_msgs::msg::Marker link_marker;
    link_marker.header.frame_id = frame_id;
    link_marker.header.stamp = timestamp;
    link_marker.ns = "path_links";
    link_marker.id = 1;
    link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    link_marker.action = visualization_msgs::msg::Marker::ADD;
    link_marker.scale.x = link_scale;
    link_marker.color = createColor(1.0, 0.0, 0.0, 0.9); // Red
    link_marker.pose.orientation.w = 1.0;

    for (size_t i = 0; i < rich_result.nodeSequence.size() - 1; ++i) {
      const auto& start_node = rich_result.nodeSequence[i];
      const auto& end_node = rich_result.nodeSequence[i + 1];

      geometry_msgs::msg::Point p1, p2;
      p1.x = start_node.x;
      p1.y = start_node.y;
      p1.z = 0.02;

      p2.x = end_node.x;
      p2.y = end_node.y;
      p2.z = 0.02;

      link_marker.points.push_back(p1);
      link_marker.points.push_back(p2);
    }
    markers.markers.push_back(link_marker);
  }

  // 3. Curved path segments
  if (!rich_result.curvedLinkSequence.empty()) {
    visualization_msgs::msg::Marker curve_marker;
    curve_marker.header.frame_id = frame_id;
    curve_marker.header.stamp = timestamp;
    curve_marker.ns = "path_curves";
    curve_marker.id = 2;
    curve_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    curve_marker.action = visualization_msgs::msg::Marker::ADD;
    curve_marker.scale.x = link_scale * 1.5;
    curve_marker.color = createColor(0.0, 0.0, 1.0, 0.9); // Blue for curves
    curve_marker.pose.orientation.w = 1.0;

    for (const auto& curve_link : rich_result.curvedLinkSequence) {
      // Find corresponding start and end nodes
      auto start_it = std::find_if(rich_result.nodeSequence.begin(), rich_result.nodeSequence.end(),
          [&curve_link](const auto& node) { return node.id == curve_link.id_start; });
      auto end_it = std::find_if(rich_result.nodeSequence.begin(), rich_result.nodeSequence.end(),
          [&curve_link](const auto& node) { return node.id == curve_link.id_end; });

      if (start_it != rich_result.nodeSequence.end() && end_it != rich_result.nodeSequence.end()) {
        Eigen::Vector3d start_pose(start_it->x, start_it->y, 0.0);
        Eigen::Vector3d end_pose(end_it->x, end_it->y, 0.0);

        auto curve_poses = curve_link.interpolateCurve(start_pose, end_pose, 0.02);
        for (const auto& pose : curve_poses) {
          geometry_msgs::msg::Point point;
          point.x = pose.x();
          point.y = pose.y();
          point.z = 0.03;
          curve_marker.points.push_back(point);
        }
      }
    }
    if (!curve_marker.points.empty()) {
      markers.markers.push_back(curve_marker);
    }
  }

  // 4. Pose sequence (detailed trajectory)
  if (!rich_result.poseSequence.empty()) {
    visualization_msgs::msg::Marker pose_marker;
    pose_marker.header.frame_id = frame_id;
    pose_marker.header.stamp = timestamp;
    pose_marker.ns = "pose_sequence";
    pose_marker.id = 3;
    pose_marker.type = visualization_msgs::msg::Marker::ARROW;
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.scale.x = 0.1; // Length
    pose_marker.scale.y = 0.02; // Width
    pose_marker.scale.z = 0.02; // Height
    pose_marker.color = createColor(1.0, 1.0, 0.0, 0.7); // Yellow
    
    // Only show every 10th pose to avoid clutter
    for (size_t i = 0; i < rich_result.poseSequence.size(); i += 10) {
      const auto& pose = rich_result.poseSequence[i];
      
      pose_marker.id = 3 + i;
      pose_marker.pose.position.x = pose.x();
      pose_marker.pose.position.y = pose.y();
      pose_marker.pose.position.z = 0.01;
      
      // Convert yaw to quaternion
      pose_marker.pose.orientation.x = 0.0;
      pose_marker.pose.orientation.y = 0.0;
      pose_marker.pose.orientation.z = std::sin(pose.z() / 2.0);
      pose_marker.pose.orientation.w = std::cos(pose.z() / 2.0);
      
      markers.markers.push_back(pose_marker);
    }
  }

  return markers;
}

visualization_msgs::msg::MarkerArray RichVisualization::createMapMarkers(
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::CurveLinkInfo>& curved_links,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double node_scale,
    double link_scale) {

  visualization_msgs::msg::MarkerArray markers;

  // Add nodes markers
  addNodesMarkers(markers, nodes, frame_id, timestamp, node_scale);
  
  // Add straight links markers
  addLinksMarkers(markers, links, nodes, frame_id, timestamp, link_scale);
  
  // Add curved links markers
  addCurvedLinksMarkers(markers, curved_links, nodes, frame_id, timestamp, link_scale);

  return markers;
}

std_msgs::msg::ColorRGBA RichVisualization::createColor(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::msg::MarkerArray RichVisualization::createClearMarkers(
    const std::string& frame_id,
    const rclcpp::Time& timestamp) {
  
  visualization_msgs::msg::MarkerArray markers;
  
  // Create a single delete-all marker
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = frame_id;
  clear_marker.header.stamp = timestamp;
  clear_marker.ns = "";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  
  markers.markers.push_back(clear_marker);
  return markers;
}

// ========================================================================
// PRIVATE HELPER METHODS
// ========================================================================

void RichVisualization::addNodesMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double scale) {

  if (nodes.empty()) return;

  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = frame_id;
  node_marker.header.stamp = timestamp;
  node_marker.ns = "map_nodes";
  node_marker.id = 0;
  node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.scale.x = scale;
  node_marker.scale.y = scale;
  node_marker.scale.z = scale;
  node_marker.color = createColor(0.0, 0.0, 1.0, 0.6); // Blue for nodes
  node_marker.pose.orientation.w = 1.0;

  for (const auto& [node_id, node_info] : nodes) {
    geometry_msgs::msg::Point point;
    point.x = node_info.x;
    point.y = node_info.y;
    point.z = 0.0;
    node_marker.points.push_back(point);
  }

  markers.markers.push_back(node_marker);
}

void RichVisualization::addLinksMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::LinkInfo>& links,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double scale) {

  if (links.empty()) return;

  visualization_msgs::msg::Marker link_marker;
  link_marker.header.frame_id = frame_id;
  link_marker.header.stamp = timestamp;
  link_marker.ns = "map_links";
  link_marker.id = 1;
  link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  link_marker.action = visualization_msgs::msg::Marker::ADD;
  link_marker.scale.x = scale;
  link_marker.color = createColor(0.5, 0.5, 0.5, 0.6); // Gray for straight links
  link_marker.pose.orientation.w = 1.0;

  for (const auto& [link_id, link_info] : links) {
    auto start_it = nodes.find(link_info.id_start);
    auto end_it = nodes.find(link_info.id_end);
    
    if (start_it != nodes.end() && end_it != nodes.end()) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = start_it->second.x;
      p1.y = start_it->second.y;
      p1.z = 0.0;
      
      p2.x = end_it->second.x;
      p2.y = end_it->second.y;
      p2.z = 0.0;
      
      link_marker.points.push_back(p1);
      link_marker.points.push_back(p2);
    }
  }

  markers.markers.push_back(link_marker);
}

void RichVisualization::addCurvedLinksMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::CurveLinkInfo>& curved_links,
    const std::unordered_map<int32_t, vrobot_route_follow::data_structures::NodeInfo>& nodes,
    const std::string& frame_id,
    const rclcpp::Time& timestamp,
    double scale) {

  if (curved_links.empty()) return;

  // Create curved links marker
  visualization_msgs::msg::Marker curved_link_marker;
  curved_link_marker.header.frame_id = frame_id;
  curved_link_marker.header.stamp = timestamp;
  curved_link_marker.ns = "curved_links";
  curved_link_marker.id = 2;
  curved_link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  curved_link_marker.action = visualization_msgs::msg::Marker::ADD;
  curved_link_marker.scale.x = scale * 1.5; // Thicker for curves
  curved_link_marker.color = createColor(0.2, 0.8, 0.2, 0.8); // Green for curves
  curved_link_marker.pose.orientation.w = 1.0;

  // Add curved links as interpolated line segments
  for (const auto& [curve_id, curve_info] : curved_links) {
    auto start_it = nodes.find(curve_info.id_start);
    auto end_it = nodes.find(curve_info.id_end);
    
    if (start_it != nodes.end() && end_it != nodes.end()) {
      const auto& start_node = start_it->second;
      const auto& end_node = end_it->second;
      
      Eigen::Vector3d start_pose(start_node.x, start_node.y, 0.0);
      Eigen::Vector3d end_pose(end_node.x, end_node.y, 0.0);
      
      // Interpolate curve with fine resolution for smooth visualization
      auto curve_poses = curve_info.interpolateCurve(start_pose, end_pose, 0.05);
      
      // Add line segments between consecutive points
      for (size_t i = 0; i < curve_poses.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = curve_poses[i].x();
        p1.y = curve_poses[i].y();
        p1.z = 0.05; // Slightly elevated
        
        p2.x = curve_poses[i + 1].x();
        p2.y = curve_poses[i + 1].y();
        p2.z = 0.05;
        
        curved_link_marker.points.push_back(p1);
        curved_link_marker.points.push_back(p2);
      }
    }
  }

  // Control points marker (for debugging)
  visualization_msgs::msg::Marker control_points_marker;
  control_points_marker.header.frame_id = frame_id;
  control_points_marker.header.stamp = timestamp;
  control_points_marker.ns = "control_points";
  control_points_marker.id = 3;
  control_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  control_points_marker.action = visualization_msgs::msg::Marker::ADD;
  control_points_marker.scale.x = control_points_marker.scale.y = control_points_marker.scale.z = scale * 0.5;
  control_points_marker.color = createColor(1.0, 0.5, 0.0, 0.7); // Orange for control points
  control_points_marker.pose.orientation.w = 1.0;

  for (const auto& [curve_id, curve_info] : curved_links) {
    geometry_msgs::msg::Point cp1, cp2;
    cp1.x = curve_info.control_point_1.x();
    cp1.y = curve_info.control_point_1.y();
    cp1.z = 0.1;
    
    cp2.x = curve_info.control_point_2.x();
    cp2.y = curve_info.control_point_2.y();
    cp2.z = 0.1;
    
    control_points_marker.points.push_back(cp1);
    control_points_marker.points.push_back(cp2);
  }

  markers.markers.push_back(curved_link_marker);
  if (!control_points_marker.points.empty()) {
    markers.markers.push_back(control_points_marker);
  }
}

} // namespace utils
} // namespace vrobot_route_follow