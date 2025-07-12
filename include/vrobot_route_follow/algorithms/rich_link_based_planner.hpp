#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "vrobot_route_follow/algorithms/rich_pathfinding.hpp"
#include "vrobot_route_follow/core/rich_geometric_utils.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"

namespace vrobot_route_follow {
namespace algorithms {

// Type aliases for convenience
using NodeInfo       = data_structures::NodeInfo;
using LinkInfo       = data_structures::LinkInfo;
using RichPathResult = data_structures::RichPathResult;

/**
 * @brief Configuration for link-based planning
 */
struct LinkBasedPlanningConfig {
  // Connection parameters
  double max_connection_distance = 5.0;
  double goal_tolerance          = 0.5;
  double projection_tolerance    = 0.2;

  // Path quality preferences
  bool   prefer_direct_connections = true;
  bool   optimize_path_smoothness  = true;
  double smoothness_weight         = 0.3;

  // Link selection criteria
  bool   prefer_shorter_links  = true;
  bool   avoid_congested_links = false;
  double congestion_penalty    = 1.5;

  // Fallback options
  bool   use_dijkstra_fallback = true;
  bool   use_astar_fallback    = true;
  size_t max_fallback_attempts = 3;

  // Custom link scoring function
  std::function<double(const LinkInfo &, const Eigen::Vector3d &,
                       const Eigen::Vector3d &)>
      custom_link_scorer;
};

/**
 * @brief Link connection information
 */
struct LinkConnection {
  int32_t         link_id;
  Eigen::Vector3d connection_point;
  double          connection_distance;
  double          projection_parameter; // t ∈ [0,1] for parametric projection
  bool            is_direct_connection;

  LinkConnection(int32_t id, const Eigen::Vector3d &point, double dist,
                 double param = 0.0, bool direct = false)
      : link_id(id), connection_point(point), connection_distance(dist),
        projection_parameter(param), is_direct_connection(direct) {}
};

/**
 * @brief Path segment with link information
 */
struct LinkPathSegment {
  Eigen::Vector3d        start_pose;
  Eigen::Vector3d        end_pose;
  std::optional<int32_t> link_id;
  std::optional<int32_t> start_node_id;
  std::optional<int32_t> end_node_id;
  double                 segment_length;
  std::string segment_type; // "direct", "link_follow", "node_connection"

  LinkPathSegment(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                  const std::string &type = "direct")
      : start_pose(start), end_pose(end), segment_type(type) {
    segment_length = (end_pose.head<2>() - start_pose.head<2>()).norm();
  }
};

/**
 * @brief Rich Link-Based Planner
 */
class RichLinkBasedPlanner {
private:
  std::shared_ptr<GraphDataInterface> graph_data_;
  std::shared_ptr<RichPathfinding>    pathfinding_;

  // Helper methods for link analysis
  std::vector<LinkConnection>
  findNearbyLinks(const Eigen::Vector3d         &pose,
                  const LinkBasedPlanningConfig &config) const;

  LinkConnection
  findBestLinkConnection(const Eigen::Vector3d             &pose,
                         const std::vector<LinkConnection> &candidates,
                         const LinkBasedPlanningConfig     &config) const;

  Eigen::Vector3d projectPointOntoLink(const Eigen::Vector3d &point,
                                       const LinkInfo        &link) const;

  double calculateLinkScore(const LinkInfo                &link,
                            const Eigen::Vector3d         &start_pose,
                            const Eigen::Vector3d         &target_pose,
                            const LinkBasedPlanningConfig &config) const;

  // Path construction methods
  std::vector<LinkPathSegment>
  buildLinkPath(const LinkConnection          &start_connection,
                const LinkConnection          &target_connection,
                const LinkBasedPlanningConfig &config) const;

  std::vector<LinkPathSegment>
  buildDirectPath(const Eigen::Vector3d &start_pose,
                  const Eigen::Vector3d &target_pose) const;

  RichPathResult
  convertToRichPathResult(const std::vector<LinkPathSegment> &segments,
                          const std::string &algorithm_name) const;

  // Validation and optimization
  bool validateLinkPath(const std::vector<LinkPathSegment> &segments) const;
  std::vector<LinkPathSegment>
  optimizePath(const std::vector<LinkPathSegment> &segments,
               const LinkBasedPlanningConfig      &config) const;

  // Fallback methods
  RichPathResult
  tryNodeBasedFallback(const Eigen::Vector3d         &start_pose,
                       const Eigen::Vector3d         &target_pose,
                       const LinkBasedPlanningConfig &config) const;

public:
  /**
   * @brief Constructor
   * @param graph_data Graph data interface
   * @param pathfinding Pathfinding instance for fallback
   */
  RichLinkBasedPlanner(std::shared_ptr<GraphDataInterface> graph_data,
                       std::shared_ptr<RichPathfinding> pathfinding = nullptr);

  /**
   * @brief Plan path using link-based approach
   * @param start_pose Starting pose
   * @param target_pose Target pose
   * @param config Configuration for link-based planning
   * @return Rich path result with full traceability
   */
  RichPathResult planPath(
      const Eigen::Vector3d &start_pose, const Eigen::Vector3d &target_pose,
      const LinkBasedPlanningConfig &config = LinkBasedPlanningConfig{}) const;

  /**
   * @brief Plan path from pose to specific node
   * @param start_pose Starting pose
   * @param target_node Target node ID
   * @param config Configuration for link-based planning
   * @return Rich path result with full traceability
   */
  RichPathResult planPathToNode(
      const Eigen::Vector3d &start_pose, int32_t target_node,
      const LinkBasedPlanningConfig &config = LinkBasedPlanningConfig{}) const;

  /**
   * @brief Plan path from node to pose
   * @param start_node Starting node ID
   * @param target_pose Target pose
   * @param config Configuration for link-based planning
   * @return Rich path result with full traceability
   */
  RichPathResult planPathFromNode(
      int32_t start_node, const Eigen::Vector3d &target_pose,
      const LinkBasedPlanningConfig &config = LinkBasedPlanningConfig{}) const;

  /**
   * @brief Get planning statistics for last operation
   */
  struct LinkPlanningStats {
    size_t      links_evaluated     = 0;
    size_t      connections_tested  = 0;
    double      computation_time_ms = 0.0;
    bool        used_fallback       = false;
    std::string fallback_algorithm;
    std::string planning_strategy; // "direct", "single_link", "multi_link",
                                   // "node_based"
  };

  LinkPlanningStats getLastStats() const { return last_stats_; }

  /**
   * @brief Analyze link connectivity for debugging
   * @param pose Query pose
   * @param config Configuration
   * @return Vector of link connections sorted by score
   */
  std::vector<LinkConnection> analyzeLinkConnectivity(
      const Eigen::Vector3d         &pose,
      const LinkBasedPlanningConfig &config = LinkBasedPlanningConfig{}) const;

private:
  mutable LinkPlanningStats last_stats_;
};

/**
 * @brief Factory for creating link-based planner instances
 */
class LinkBasedPlannerFactory {
public:
  /**
   * @brief Create link-based planner with graph data
   * @param graph_data Graph data interface
   * @param pathfinding Optional pathfinding instance for fallback
   * @return Unique pointer to link-based planner
   */
  static std::unique_ptr<RichLinkBasedPlanner>
  create(std::shared_ptr<GraphDataInterface> graph_data,
         std::shared_ptr<RichPathfinding>    pathfinding = nullptr);

  /**
   * @brief Create default link-based planning configuration
   * @return Default configuration
   */
  static LinkBasedPlanningConfig createDefaultConfig();

  /**
   * @brief Create configuration optimized for direct connections
   * @return Direct connection optimized configuration
   */
  static LinkBasedPlanningConfig createDirectConnectionConfig();

  /**
   * @brief Create configuration optimized for link following
   * @return Link following optimized configuration
   */
  static LinkBasedPlanningConfig createLinkFollowingConfig();

  /**
   * @brief Create configuration with aggressive fallback
   * @return Fallback-heavy configuration
   */
  static LinkBasedPlanningConfig createRobustConfig();
};

/**
 * @brief Link-based planning utilities
 */
namespace link_planning_utils {

/**
 * @brief Calculate optimal connection point on link
 * @param pose Query pose
 * @param link Link information
 * @param start_node Start node of link
 * @param end_node End node of link
 * @return Optimal connection point and projection parameter
 */
std::pair<Eigen::Vector3d, double> calculateOptimalConnectionPoint(
    const Eigen::Vector3d &pose, const LinkInfo &link,
    const NodeInfo &start_node, const NodeInfo &end_node);

/**
 * @brief Check if two poses can be connected directly
 * @param pose1 First pose
 * @param pose2 Second pose
 * @param max_distance Maximum connection distance
 * @param max_angle_diff Maximum angle difference (radians)
 * @return True if direct connection is feasible
 */
bool canConnectDirectly(const Eigen::Vector3d &pose1,
                        const Eigen::Vector3d &pose2, double max_distance = 5.0,
                        double max_angle_diff = M_PI / 2);

/**
 * @brief Smooth path by removing unnecessary waypoints
 * @param segments Input path segments
 * @param smoothing_tolerance Tolerance for smoothing
 * @return Smoothed path segments
 */
std::vector<LinkPathSegment>
smoothPath(const std::vector<LinkPathSegment> &segments,
           double                              smoothing_tolerance = 0.1);

/**
 * @brief Validate path continuity and feasibility
 * @param segments Path segments to validate
 * @param max_gap Maximum allowed gap between segments
 * @return Validation result with error messages
 */
struct PathValidationResult {
  bool                     is_valid = true;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

PathValidationResult validatePath(const std::vector<LinkPathSegment> &segments,
                                  double max_gap = 0.1);

} // namespace link_planning_utils

} // namespace algorithms
} // namespace vrobot_route_follow