#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "vrobot_route_follow/core/rich_geometric_utils.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"

using vrobot_route_follow::data_structures::LinkInfo;
using vrobot_route_follow::data_structures::NodeInfo;
using vrobot_route_follow::data_structures::RichPathResult;

namespace vrobot_route_follow {
namespace algorithms {

/**
 * @brief Configuration for pathfinding algorithms
 */
struct PathfindingConfig {
  // Distance constraints
  double                max_connection_distance = 5.0;
  double                goal_tolerance          = 0.5;
  std::optional<double> max_search_distance;
  std::optional<size_t> max_nodes_explored;

  // Path quality preferences
  bool   prefer_shorter_paths = true;
  bool   avoid_sharp_turns    = false;
  double turn_penalty_factor  = 1.0;

  // Velocity constraints
  std::optional<double> max_velocity;
  std::optional<double> min_velocity;

  // Custom scoring function
  std::function<double(const LinkInfo &, const NodeInfo &, const NodeInfo &)>
      custom_scorer;

  // A* specific parameters
  double heuristic_weight        = 1.0;
  bool   use_euclidean_heuristic = true;
};

/**
 * @brief Graph data interface for pathfinding algorithms
 */
class GraphDataInterface {
public:
  virtual ~GraphDataInterface() = default;

  // Node access
  virtual const NodeInfo      &getNode(int32_t node_id) const = 0;
  virtual bool                 hasNode(int32_t node_id) const = 0;
  virtual std::vector<int32_t> getAllNodeIds() const          = 0;

  // Link access
  virtual const LinkInfo      &getLink(int32_t link_id) const              = 0;
  virtual bool                 hasLink(int32_t link_id) const              = 0;
  virtual std::vector<int32_t> getNodeNeighborLinks(int32_t node_id) const = 0;

  // Geometric queries
  virtual int32_t findClosestNode(const Eigen::Vector3d &pose) const  = 0;
  virtual std::vector<int32_t> findNodesInRadius(const Eigen::Vector3d &center,
                                                 double radius) const = 0;
};

/**
 * @brief Priority queue element for pathfinding
 */
struct PathfindingNode {
  int32_t node_id;
  double  cost;
  double  heuristic;
  int32_t parent_node_id;
  int32_t parent_link_id;

  // Default constructor for std::unordered_map
  PathfindingNode()
      : node_id(-1), cost(0.0), heuristic(0.0), parent_node_id(-1),
        parent_link_id(-1) {}

  PathfindingNode(int32_t id, double c, double h = 0.0,
                  int32_t parent_node = -1, int32_t parent_link = -1)
      : node_id(id), cost(c), heuristic(h), parent_node_id(parent_node),
        parent_link_id(parent_link) {}

  double getTotalCost() const { return cost + heuristic; }

  // For priority queue (min-heap)
  bool operator>(const PathfindingNode &other) const {
    return getTotalCost() > other.getTotalCost();
  }
};

/**
 * @brief Rich Pathfinding class with multiple algorithms
 */
class RichPathfinding {
private:
  std::shared_ptr<GraphDataInterface> graph_data_;

  // Helper methods
  double calculateLinkCost(const LinkInfo &link, const NodeInfo &start_node,
                           const NodeInfo          &end_node,
                           const PathfindingConfig &config) const;
  double calculateHeuristic(const NodeInfo &current, const NodeInfo &target,
                            const PathfindingConfig &config) const;
  RichPathResult
  reconstructPath(const std::unordered_map<int32_t, PathfindingNode> &came_from,
                  int32_t start_node, int32_t target_node,
                  const std::string &algorithm_name) const;

  // Validation helpers
  bool validateNodes(int32_t start_node, int32_t target_node) const;
  bool shouldTerminateSearch(const PathfindingNode   &current,
                             const PathfindingConfig &config,
                             size_t                   nodes_explored) const;

public:
  /**
   * @brief Constructor
   * @param graph_data Shared pointer to graph data interface
   */
  explicit RichPathfinding(std::shared_ptr<GraphDataInterface> graph_data);

  /**
   * @brief Dijkstra's algorithm for shortest path
   * @param start_node Starting node ID
   * @param target_node Target node ID
   * @param config Configuration for pathfinding
   * @return Rich path result with full traceability
   */
  RichPathResult
  planDijkstra(int32_t start_node, int32_t target_node,
               const PathfindingConfig &config = PathfindingConfig{}) const;

  /**
   * @brief A* algorithm for shortest path with heuristic
   * @param start_node Starting node ID
   * @param target_node Target node ID
   * @param config Configuration for pathfinding
   * @return Rich path result with full traceability
   */
  RichPathResult
  planAStar(int32_t start_node, int32_t target_node,
            const PathfindingConfig &config = PathfindingConfig{}) const;

  /**
   * @brief Direct path planning (straight line connection)
   * @param start_pose Starting pose
   * @param target_node Target node ID
   * @param config Configuration for pathfinding
   * @return Rich path result with direct connection
   */
  RichPathResult
  planDirectPath(const Eigen::Vector3d &start_pose, int32_t target_node,
                 const PathfindingConfig &config = PathfindingConfig{}) const;

  /**
   * @brief Get pathfinding statistics for last operation
   * @return Statistics including nodes explored, time taken, etc.
   */
  struct PathfindingStats {
    size_t      nodes_explored          = 0;
    size_t      links_evaluated         = 0;
    double      computation_time_ms     = 0.0;
    bool        search_terminated_early = false;
    std::string termination_reason;
  };

  PathfindingStats getLastStats() const { return last_stats_; }

private:
  mutable PathfindingStats last_stats_;
};

/**
 * @brief Factory for creating pathfinding instances
 */
class PathfindingFactory {
public:
  /**
   * @brief Create pathfinding instance with graph data
   * @param graph_data Graph data interface
   * @return Unique pointer to pathfinding instance
   */
  static std::unique_ptr<RichPathfinding>
  create(std::shared_ptr<GraphDataInterface> graph_data);

  /**
   * @brief Create default pathfinding configuration
   * @return Default configuration
   */
  static PathfindingConfig createDefaultConfig();

  /**
   * @brief Create configuration optimized for speed
   * @return Speed-optimized configuration
   */
  static PathfindingConfig createSpeedOptimizedConfig();

  /**
   * @brief Create configuration optimized for accuracy
   * @return Accuracy-optimized configuration
   */
  static PathfindingConfig createAccuracyOptimizedConfig();
};

} // namespace algorithms
} // namespace vrobot_route_follow