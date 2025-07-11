#pragma once

#include "../data_structures/link_info.hpp"
#include "../data_structures/node_info.hpp"
#include "../data_structures/rich_path_result.hpp"
#include <memory>
#include <unordered_map>
#include <vector>

// Forward declarations for database ORM
namespace drogon {
namespace orm {
class DbClient;
}
} // namespace drogon

namespace drogon_model {
namespace amr_01 {
namespace amr_ros2 {
class Node;
class Straightlink;
class Map;
} // namespace amr_ros2
} // namespace amr_01
} // namespace drogon_model

// Forward declarations for ROS messages
namespace nav_msgs {
namespace msg {
class Path;
}
} // namespace nav_msgs

namespace vrobot_local_planner {
namespace msg {
class Path;
}
} // namespace vrobot_local_planner

namespace geometry_msgs {
namespace msg {
class PoseStamped;
}
} // namespace geometry_msgs

namespace rclcpp {
class Time;
}

namespace vrobot_route_follow {
namespace utils {

/**
 * @brief Database converter utilities for converting between ORM objects and
 * data structures
 *
 * This class provides static methods to convert between database ORM objects
 * and the new rich data structures, as well as conversion to ROS messages.
 */
class DatabaseConverter {
public:
  // ========================================================================
  // DATABASE TO DATA STRUCTURES
  // ========================================================================

  /**
   * @brief Convert database Node to NodeInfo
   */
  static data_structures::NodeInfo
  convertNode(const drogon_model::amr_01::amr_ros2::Node &db_node);

  /**
   * @brief Convert database Straightlink to LinkInfo
   */
  static data_structures::LinkInfo
  convertLink(const drogon_model::amr_01::amr_ros2::Straightlink &db_link);

  /**
   * @brief Convert multiple database nodes to NodeInfo vector
   */
  static std::vector<data_structures::NodeInfo> convertNodes(
      const std::vector<drogon_model::amr_01::amr_ros2::Node> &db_nodes);

  /**
   * @brief Convert multiple database links to LinkInfo vector
   */
  static std::vector<data_structures::LinkInfo>
  convertLinks(const std::vector<drogon_model::amr_01::amr_ros2::Straightlink>
                   &db_links);

  /**
   * @brief Create node lookup map from database nodes
   */
  static std::unordered_map<int32_t, data_structures::NodeInfo> createNodeMap(
      const std::vector<drogon_model::amr_01::amr_ros2::Node> &db_nodes);

  /**
   * @brief Create link lookup map from database links
   */
  static std::unordered_map<int32_t, data_structures::LinkInfo>
  createLinkMap(const std::vector<drogon_model::amr_01::amr_ros2::Straightlink>
                    &db_links);

  // ========================================================================
  // DATA STRUCTURES TO ROS MESSAGES
  // ========================================================================

  /**
   * @brief Convert NodeInfo to geometry_msgs::PoseStamped
   */
  static geometry_msgs::msg::PoseStamped
  nodeToRosPose(const data_structures::NodeInfo &node,
                const std::string &frame_id, const rclcpp::Time &timestamp);

  /**
   * @brief Convert Eigen::Vector3d to geometry_msgs::PoseStamped
   */
  static geometry_msgs::msg::PoseStamped
  eigenPoseToRosPose(const Eigen::Vector3d &pose, const std::string &frame_id,
                     const rclcpp::Time &timestamp);

  /**
   * @brief Convert RichPathResult to nav_msgs::Path
   */
  static nav_msgs::msg::Path
  richPathToNavPath(const data_structures::RichPathResult &rich_path,
                    const std::string &frame_id, const rclcpp::Time &timestamp,
                    double interpolation_resolution = 0.02);

  /**
   * @brief Convert RichPathResult to vrobot_local_planner::Path
   */
  static vrobot_local_planner::msg::Path
  richPathToVPath(const data_structures::RichPathResult &rich_path,
                  const std::string &frame_id, const rclcpp::Time &timestamp,
                  double interpolation_resolution = 0.02);

  // ========================================================================
  // LEGACY COMPATIBILITY
  // ========================================================================

  /**
   * @brief Convert RichPathResult to legacy PathSegment format
   */
  static std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  richPathToPathSegments(const data_structures::RichPathResult &rich_path);

  /**
   * @brief Convert legacy PathSegment format to nav_msgs::Path
   */
  static nav_msgs::msg::Path pathSegmentsToNavPath(
      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                        &path_segments,
      const std::string &frame_id, const rclcpp::Time &timestamp,
      double interpolation_resolution = 0.02);

  // ========================================================================
  // UTILITY FUNCTIONS
  // ========================================================================

  /**
   * @brief Interpolate poses between two points
   */
  static std::vector<Eigen::Vector3d>
  interpolatePoses(const Eigen::Vector3d &start_pose,
                   const Eigen::Vector3d &end_pose, double resolution);

  /**
   * @brief Calculate quaternion from 2D angle (theta)
   */
  static std::array<double, 4> angleToQuaternion(double theta);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  static double normalizeAngle(double angle);

  /**
   * @brief Create adjacency list from nodes and links
   */
  static std::unordered_map<int32_t, std::vector<int32_t>> createAdjacencyList(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  /**
   * @brief Create reverse adjacency list (for incoming links)
   */
  static std::unordered_map<int32_t, std::vector<int32_t>>
  createReverseAdjacencyList(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  // ========================================================================
  // VALIDATION UTILITIES
  // ========================================================================

  /**
   * @brief Validate node-link consistency
   */
  static bool validateNodeLinkConsistency(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  /**
   * @brief Find orphaned nodes (nodes without any links)
   */
  static std::vector<int32_t> findOrphanedNodes(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  /**
   * @brief Find invalid links (links pointing to non-existent nodes)
   */
  static std::vector<int32_t> findInvalidLinks(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  // ========================================================================
  // STATISTICS
  // ========================================================================

  /**
   * @brief Generate graph statistics
   */
  struct GraphStatistics {
    size_t                                  total_nodes     = 0;
    size_t                                  total_links     = 0;
    size_t                                  orphaned_nodes  = 0;
    size_t                                  invalid_links   = 0;
    double                                  avg_node_degree = 0.0;
    double                                  max_velocity    = 0.0;
    double                                  min_velocity    = 0.0;
    std::unordered_map<std::string, size_t> node_types;
    std::unordered_map<std::string, size_t> link_types;
  };

  /**
   * @brief Calculate graph statistics
   */
  static GraphStatistics calculateGraphStatistics(
      const std::unordered_map<int32_t, data_structures::NodeInfo> &nodes,
      const std::unordered_map<int32_t, data_structures::LinkInfo> &links);

  /**
   * @brief Get statistics as string
   */
  static std::string statisticsToString(const GraphStatistics &stats);

private:
  // Private constructor to prevent instantiation
  DatabaseConverter() = default;
};

} // namespace utils
} // namespace vrobot_route_follow