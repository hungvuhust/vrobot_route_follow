#include "vrobot_route_follow/utils/database_converter.hpp"

// Standard includes
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vrobot_route_follow {
namespace utils {

// ========================================================================
// DATABASE TO DATA STRUCTURES
// ========================================================================

data_structures::NodeInfo DatabaseConverter::convertNode(
    const drogon_model::amr_01::amr_ros2::Node &db_node) {

  data_structures::NodeInfo node_info;

  // Core database fields
  node_info.id        = db_node.getValueOfId();
  node_info.node_name = db_node.getValueOfNodeName();
  node_info.x         = db_node.getValueOfX();
  node_info.y         = db_node.getValueOfY();
  node_info.theta     = db_node.getValueOfTheta();
  node_info.type      = db_node.getValueOfType();
  node_info.map_id    = db_node.getValueOfMapId();

  // Extended attributes can be set based on node type or other logic
  if (node_info.type == "charging" || node_info.type == "charge_station") {
    node_info.is_charging_station = true;
  }

  // Set zone type based on node type
  if (node_info.type == "pickup" || node_info.type == "dropoff") {
    node_info.zone_type = "operation_zone";
  } else if (node_info.type == "waiting") {
    node_info.zone_type        = "waiting_zone";
    node_info.max_waiting_time = 300.0; // 5 minutes default
  }

  return node_info;
}

data_structures::LinkInfo DatabaseConverter::convertLink(
    const drogon_model::amr_01::amr_ros2::Straightlink &db_link) {

  data_structures::LinkInfo link_info;

  // Core database fields
  link_info.id_straight_link = db_link.getValueOfIdStraightLink();
  link_info.id_start         = db_link.getValueOfIdStart();
  link_info.id_end           = db_link.getValueOfIdEnd();
  link_info.map_id           = db_link.getValueOfMapId();
  link_info.max_velocity     = db_link.getValueOfMaxVelocity();

  // Extended attributes - set defaults
  link_info.bidirectional     = true; // Default assumption
  link_info.traffic_direction = "both";
  link_info.link_type         = "normal";
  link_info.width             = 1.0; // Default width
  link_info.is_emergency_path = false;
  link_info.priority          = 1.0;

  // Set minimum velocity based on max velocity
  if (link_info.max_velocity > 0) {
    link_info.min_velocity = std::min(0.1, link_info.max_velocity * 0.1);
  }

  return link_info;
}

std::vector<data_structures::NodeInfo> DatabaseConverter::convertNodes(
    const std::vector<drogon_model::amr_01::amr_ros2::Node> &db_nodes) {

  std::vector<data_structures::NodeInfo> nodes;
  nodes.reserve(db_nodes.size());

  for (const auto &db_node : db_nodes) {
    nodes.push_back(convertNode(db_node));
  }

  return nodes;
}

std::vector<data_structures::LinkInfo> DatabaseConverter::convertLinks(
    const std::vector<drogon_model::amr_01::amr_ros2::Straightlink> &db_links) {

  std::vector<data_structures::LinkInfo> links;
  links.reserve(db_links.size());

  for (const auto &db_link : db_links) {
    links.push_back(convertLink(db_link));
  }

  return links;
}

std::unordered_map<int32_t, data_structures::NodeInfo>
DatabaseConverter::createNodeMap(
    const std::vector<drogon_model::amr_01::amr_ros2::Node> &db_nodes) {

  std::unordered_map<int32_t, data_structures::NodeInfo> node_map;

  for (const auto &db_node : db_nodes) {
    auto node_info         = convertNode(db_node);
    node_map[node_info.id] = std::move(node_info);
  }

  return node_map;
}

std::unordered_map<int32_t, data_structures::LinkInfo>
DatabaseConverter::createLinkMap(
    const std::vector<drogon_model::amr_01::amr_ros2::Straightlink> &db_links) {

  std::unordered_map<int32_t, data_structures::LinkInfo> link_map;

  for (const auto &db_link : db_links) {
    auto link_info                       = convertLink(db_link);
    link_map[link_info.id_straight_link] = std::move(link_info);
  }

  return link_map;
}

// ========================================================================
// DATA STRUCTURES TO ROS MESSAGES
// ========================================================================

geometry_msgs::msg::PoseStamped
DatabaseConverter::nodeToRosPose(const data_structures::NodeInfo &node,
                                 const std::string               &frame_id,
                                 const rclcpp::Time              &timestamp) {

  geometry_msgs::msg::PoseStamped pose_stamped;

  // Header
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp    = timestamp;

  // Position
  pose_stamped.pose.position.x = node.x;
  pose_stamped.pose.position.y = node.y;
  pose_stamped.pose.position.z = 0.0;

  // Orientation from theta
  auto quat                       = angleToQuaternion(node.theta);
  pose_stamped.pose.orientation.x = quat[0];
  pose_stamped.pose.orientation.y = quat[1];
  pose_stamped.pose.orientation.z = quat[2];
  pose_stamped.pose.orientation.w = quat[3];

  return pose_stamped;
}

geometry_msgs::msg::PoseStamped
DatabaseConverter::eigenPoseToRosPose(const Eigen::Vector3d &pose,
                                      const std::string     &frame_id,
                                      const rclcpp::Time    &timestamp) {

  geometry_msgs::msg::PoseStamped pose_stamped;

  // Header
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp    = timestamp;

  // Position
  pose_stamped.pose.position.x = pose(0);
  pose_stamped.pose.position.y = pose(1);
  pose_stamped.pose.position.z = 0.0;

  // Orientation from theta
  auto quat                       = angleToQuaternion(pose(2));
  pose_stamped.pose.orientation.x = quat[0];
  pose_stamped.pose.orientation.y = quat[1];
  pose_stamped.pose.orientation.z = quat[2];
  pose_stamped.pose.orientation.w = quat[3];

  return pose_stamped;
}

nav_msgs::msg::Path DatabaseConverter::richPathToNavPath(
    const data_structures::RichPathResult &rich_path,
    const std::string &frame_id, const rclcpp::Time &timestamp,
    double interpolation_resolution) {

  nav_msgs::msg::Path nav_path;

  // Header
  nav_path.header.frame_id = frame_id;
  nav_path.header.stamp    = timestamp;

  if (!rich_path.success || rich_path.nodeSequence.empty()) {
    return nav_path; // Return empty path
  }

  // Use interpolated poses if available, otherwise interpolate from nodes
  if (!rich_path.poseSequence.empty()) {
    // Use pre-computed pose sequence
    nav_path.poses.reserve(rich_path.poseSequence.size());
    for (const auto &pose : rich_path.poseSequence) {
      nav_path.poses.push_back(eigenPoseToRosPose(pose, frame_id, timestamp));
    }
  } else {
    // Interpolate from node sequence
    for (size_t i = 0; i < rich_path.nodeSequence.size() - 1; ++i) {
      const auto &start_node = rich_path.nodeSequence[i];
      const auto &end_node   = rich_path.nodeSequence[i + 1];

      Eigen::Vector3d start_pose = start_node.toPose();
      Eigen::Vector3d end_pose   = end_node.toPose();

      auto interpolated_poses =
          interpolatePoses(start_pose, end_pose, interpolation_resolution);

      for (const auto &pose : interpolated_poses) {
        nav_path.poses.push_back(eigenPoseToRosPose(pose, frame_id, timestamp));
      }
    }

    // Add final pose
    if (!rich_path.nodeSequence.empty()) {
      const auto &final_node = rich_path.nodeSequence.back();
      nav_path.poses.push_back(nodeToRosPose(final_node, frame_id, timestamp));
    }
  }

  return nav_path;
}

vrobot_local_planner::msg::Path DatabaseConverter::richPathToVPath(
    const data_structures::RichPathResult &rich_path,
    const std::string &frame_id, const rclcpp::Time &timestamp,
    double interpolation_resolution) {

  vrobot_local_planner::msg::Path vpath;

  // Header
  vpath.header.frame_id = frame_id;
  vpath.header.stamp    = timestamp;

  if (!rich_path.success || rich_path.nodeSequence.empty()) {
    return vpath; // Return empty path
  }

  // Triển khai giống như toVPath trong nav_conversion.cpp
  // Tạo VPathSegments từ RichPathResult
  std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Vector3d>, double>>
      vpath_segments;

  if (!rich_path.linkSequence.empty() && rich_path.nodeSequence.size() > 1) {
    // Tạo segments từ link sequence
    for (size_t i = 0; i < rich_path.linkSequence.size(); ++i) {
      const auto &link       = rich_path.linkSequence[i];
      const auto &start_node = rich_path.nodeSequence[i];
      const auto &end_node   = rich_path.nodeSequence[i + 1];

      std::pair<Eigen::Vector3d, Eigen::Vector3d> path_segment = {
          start_node.toPose(), end_node.toPose()};

      double speed = link.max_velocity;
      vpath_segments.push_back({path_segment, speed});
    }
  } else if (rich_path.nodeSequence.size() > 1) {
    // Fallback: tạo segments từ node sequence với default speed
    for (size_t i = 0; i < rich_path.nodeSequence.size() - 1; ++i) {
      const auto &start_node = rich_path.nodeSequence[i];
      const auto &end_node   = rich_path.nodeSequence[i + 1];

      std::pair<Eigen::Vector3d, Eigen::Vector3d> path_segment = {
          start_node.toPose(), end_node.toPose()};

      double speed = 0.5; // Default speed
      vpath_segments.push_back({path_segment, speed});
    }
  }

  // Triển khai giống toVPath: interpolate từng segment và gán speed
  for (const auto &segment : vpath_segments) {
    const auto &path_segment = segment.first;
    double      speed        = segment.second;

    // Interpolate poses cho segment này
    auto interpolated_poses = interpolatePoses(
        path_segment.first, path_segment.second, interpolation_resolution);

    for (const auto &pose : interpolated_poses) {
      vrobot_local_planner::msg::PlannerPose planner_pose;
      planner_pose.header.frame_id = frame_id;
      planner_pose.header.stamp    = timestamp;

      planner_pose.pose.position.x = pose(0);
      planner_pose.pose.position.y = pose(1);
      planner_pose.pose.position.z = 0.0;

      // Convert angle to quaternion
      auto quat                       = angleToQuaternion(pose(2));
      planner_pose.pose.orientation.x = quat[0];
      planner_pose.pose.orientation.y = quat[1];
      planner_pose.pose.orientation.z = quat[2];
      planner_pose.pose.orientation.w = quat[3];

      // Gán speed giống như toVPath
      planner_pose.speed = speed;

      vpath.poses.push_back(planner_pose);
    }
  }

  return vpath;
}

// ========================================================================
// LEGACY COMPATIBILITY
// ========================================================================

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
DatabaseConverter::richPathToPathSegments(
    const data_structures::RichPathResult &rich_path) {

  return rich_path.toPathSegments();
}

nav_msgs::msg::Path DatabaseConverter::pathSegmentsToNavPath(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                      &path_segments,
    const std::string &frame_id, const rclcpp::Time &timestamp,
    double interpolation_resolution) {

  nav_msgs::msg::Path nav_path;

  // Header
  nav_path.header.frame_id = frame_id;
  nav_path.header.stamp    = timestamp;

  if (path_segments.empty()) {
    return nav_path;
  }

  // Add first pose
  nav_path.poses.push_back(
      eigenPoseToRosPose(path_segments[0].first, frame_id, timestamp));

  // Interpolate each segment
  for (const auto &segment : path_segments) {
    auto interpolated_poses = interpolatePoses(segment.first, segment.second,
                                               interpolation_resolution);

    for (size_t i = 1; i < interpolated_poses.size();
         ++i) { // Skip first pose to avoid duplication
      nav_path.poses.push_back(
          eigenPoseToRosPose(interpolated_poses[i], frame_id, timestamp));
    }
  }

  return nav_path;
}

// ========================================================================
// UTILITY FUNCTIONS
// ========================================================================

std::vector<Eigen::Vector3d>
DatabaseConverter::interpolatePoses(const Eigen::Vector3d &start_pose,
                                    const Eigen::Vector3d &end_pose,
                                    double                 resolution) {

  std::vector<Eigen::Vector3d> poses;

  Eigen::Vector2d start_pos = start_pose.head<2>();
  Eigen::Vector2d end_pos   = end_pose.head<2>();

  double distance  = (end_pos - start_pos).norm();
  int    num_steps = std::max(1, static_cast<int>(distance / resolution));

  for (int i = 0; i <= num_steps; ++i) {
    double t = static_cast<double>(i) / num_steps;

    Eigen::Vector3d interpolated_pose;
    interpolated_pose.head<2>() = start_pos + t * (end_pos - start_pos);

    // Interpolate angle (handling wrap-around)
    double start_angle = normalizeAngle(start_pose(2));
    double end_angle   = normalizeAngle(end_pose(2));

    double angle_diff = end_angle - start_angle;
    if (angle_diff > M_PI) {
      angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }

    interpolated_pose(2) = normalizeAngle(start_angle + t * angle_diff);

    poses.push_back(interpolated_pose);
  }

  return poses;
}

std::array<double, 4> DatabaseConverter::angleToQuaternion(double theta) {
  double half_theta = theta * 0.5;
  double cos_half   = std::cos(half_theta);
  double sin_half   = std::sin(half_theta);

  // Quaternion for rotation around Z-axis: [x, y, z, w]
  return {0.0, 0.0, sin_half, cos_half};
}

double DatabaseConverter::normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

std::unordered_map<int32_t, std::vector<int32_t>>
DatabaseConverter::createAdjacencyList(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  std::unordered_map<int32_t, std::vector<int32_t>> adjacency_list;

  // Initialize with empty lists for all nodes
  for (const auto &[node_id, node_info] : nodes) {
    adjacency_list[node_id] = {};
  }

  // Add links to adjacency list
  for (const auto &[link_id, link_info] : links) {
    adjacency_list[link_info.id_start].push_back(link_id);

    // Add reverse link if bidirectional
    if (link_info.bidirectional.value_or(true)) {
      adjacency_list[link_info.id_end].push_back(link_id);
    }
  }

  return adjacency_list;
}

std::unordered_map<int32_t, std::vector<int32_t>>
DatabaseConverter::createReverseAdjacencyList(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  std::unordered_map<int32_t, std::vector<int32_t>> reverse_adjacency_list;

  // Initialize with empty lists for all nodes
  for (const auto &[node_id, node_info] : nodes) {
    reverse_adjacency_list[node_id] = {};
  }

  // Add incoming links to reverse adjacency list
  for (const auto &[link_id, link_info] : links) {
    reverse_adjacency_list[link_info.id_end].push_back(link_id);

    // Add reverse link if bidirectional
    if (link_info.bidirectional.value_or(true)) {
      reverse_adjacency_list[link_info.id_start].push_back(link_id);
    }
  }

  return reverse_adjacency_list;
}

// ========================================================================
// VALIDATION UTILITIES
// ========================================================================

bool DatabaseConverter::validateNodeLinkConsistency(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  for (const auto &[link_id, link_info] : links) {
    // Check if start node exists
    if (nodes.find(link_info.id_start) == nodes.end()) {
      return false;
    }

    // Check if end node exists
    if (nodes.find(link_info.id_end) == nodes.end()) {
      return false;
    }

    // Check if start and end are different
    if (link_info.id_start == link_info.id_end) {
      return false;
    }
  }

  return true;
}

std::vector<int32_t> DatabaseConverter::findOrphanedNodes(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  std::vector<int32_t> orphaned_nodes;

  for (const auto &[node_id, node_info] : nodes) {
    bool has_connection = false;

    // Check if node is connected to any link
    for (const auto &[link_id, link_info] : links) {
      if (link_info.id_start == node_id || link_info.id_end == node_id) {
        has_connection = true;
        break;
      }
    }

    if (!has_connection) {
      orphaned_nodes.push_back(node_id);
    }
  }

  return orphaned_nodes;
}

std::vector<int32_t> DatabaseConverter::findInvalidLinks(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  std::vector<int32_t> invalid_links;

  for (const auto &[link_id, link_info] : links) {
    // Check if start node exists
    bool start_exists = nodes.find(link_info.id_start) != nodes.end();

    // Check if end node exists
    bool end_exists = nodes.find(link_info.id_end) != nodes.end();

    // Check if start and end are different
    bool different_nodes = link_info.id_start != link_info.id_end;

    if (!start_exists || !end_exists || !different_nodes) {
      invalid_links.push_back(link_id);
    }
  }

  return invalid_links;
}

// ========================================================================
// STATISTICS
// ========================================================================

DatabaseConverter::GraphStatistics DatabaseConverter::calculateGraphStatistics(
    const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
    const std::unordered_map<int32_t, data_structures::LinkInfo> &links) {

  GraphStatistics stats;

  stats.total_nodes = nodes.size();
  stats.total_links = links.size();

  // Find orphaned nodes and invalid links
  auto orphaned = findOrphanedNodes(nodes, links);
  auto invalid  = findInvalidLinks(nodes, links);

  stats.orphaned_nodes = orphaned.size();
  stats.invalid_links  = invalid.size();

  // Calculate node degree statistics
  auto   adjacency_list = createAdjacencyList(nodes, links);
  double total_degree   = 0.0;

  for (const auto &[node_id, connected_links] : adjacency_list) {
    total_degree += connected_links.size();
  }

  stats.avg_node_degree =
      stats.total_nodes > 0 ? total_degree / stats.total_nodes : 0.0;

  // Calculate velocity statistics
  if (!links.empty()) {
    stats.max_velocity = 0.0;
    stats.min_velocity = std::numeric_limits<double>::max();

    for (const auto &[link_id, link_info] : links) {
      if (link_info.max_velocity > 0) {
        stats.max_velocity =
            std::max(stats.max_velocity, link_info.max_velocity);
        stats.min_velocity =
            std::min(stats.min_velocity, link_info.max_velocity);
      }
    }

    if (stats.min_velocity == std::numeric_limits<double>::max()) {
      stats.min_velocity = 0.0;
    }
  }

  // Count node types
  for (const auto &[node_id, node_info] : nodes) {
    stats.node_types[node_info.type]++;
  }

  // Count link types
  for (const auto &[link_id, link_info] : links) {
    std::string link_type = link_info.link_type.value_or("normal");
    stats.link_types[link_type]++;
  }

  return stats;
}

std::string
DatabaseConverter::statisticsToString(const GraphStatistics &stats) {
  std::string result = "Graph Statistics:\n";

  result += "  Total Nodes: " + std::to_string(stats.total_nodes) + "\n";
  result += "  Total Links: " + std::to_string(stats.total_links) + "\n";
  result += "  Orphaned Nodes: " + std::to_string(stats.orphaned_nodes) + "\n";
  result += "  Invalid Links: " + std::to_string(stats.invalid_links) + "\n";
  result +=
      "  Average Node Degree: " + std::to_string(stats.avg_node_degree) + "\n";
  result += "  Max Velocity: " + std::to_string(stats.max_velocity) + " m/s\n";
  result += "  Min Velocity: " + std::to_string(stats.min_velocity) + " m/s\n";

  result += "  Node Types:\n";
  for (const auto &[type, count] : stats.node_types) {
    result += "    " + type + ": " + std::to_string(count) + "\n";
  }

  result += "  Link Types:\n";
  for (const auto &[type, count] : stats.link_types) {
    result += "    " + type + ": " + std::to_string(count) + "\n";
  }

  return result;
}

} // namespace utils
} // namespace vrobot_route_follow