#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "vrobot_route_follow/utils/db_client.hpp"

#include "vrobot_route_follow/algorithms/algorithm_interface.hpp"
#include "vrobot_route_follow/algorithms/rich_link_based_planner.hpp"
#include "vrobot_route_follow/algorithms/rich_pathfinding.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"

using vrobot_route_follow::data_structures::LinkInfo;
using vrobot_route_follow::data_structures::NodeInfo;
using vrobot_route_follow::data_structures::RichPathResult;

namespace vrobot_route_follow {

// Forward declarations
namespace algorithms {
class AlgorithmManager;
class RichPathfinding;
class RichLinkBasedPlanner;
} // namespace algorithms

// Legacy PlanningConfig for backward compatibility
struct PlanningConfig {
  // Algorithm selection
  enum class Algorithm { DIJKSTRA, A_STAR, DIRECT_PATH, LINK_BASED };
  Algorithm algorithm = Algorithm::DIJKSTRA;

  // Distance thresholds
  double max_connection_distance =
      5.0;                     // Maximum distance to connect to nearest node
  double goal_tolerance = 0.5; // Distance tolerance to consider goal reached

  // Search constraints
  std::optional<double> max_search_distance =
      std::nullopt; // Limit search radius
  std::optional<size_t> max_nodes_explored =
      std::nullopt; // Limit computational cost

  // Path quality preferences
  bool   prefer_shorter_paths = true;
  bool   avoid_sharp_turns    = false;
  double turn_penalty_factor  = 1.0;

  // Velocity constraints
  std::optional<double> max_velocity = std::nullopt;
  std::optional<double> min_velocity = std::nullopt;

  // Custom scoring function
  std::function<double(const LinkInfo &, const NodeInfo &, const NodeInfo &)>
      custom_scorer = nullptr;

  // Convert to new AlgorithmConfig
  algorithms::AlgorithmConfig toAlgorithmConfig() const;
};

class RichGraph : public algorithms::GraphDataInterface {
private:
  // Core data structures
  std::unordered_map<int32_t, NodeInfo> nodes_;
  std::unordered_map<int32_t, LinkInfo> links_;
  std::unordered_map<int32_t, std::vector<int32_t>>
      adjacency_list_; // node_id -> list of link_ids

  // Cached data for performance
  mutable std::unordered_map<int32_t, std::vector<int32_t>>
               reverse_adjacency_list_; // node_id -> incoming link_ids
  mutable bool adjacency_cache_valid_ = false;

  // Database connection
  std::shared_ptr<drogon::orm::DbClient> db_client_;
  std::string                            current_map_name_;
  int32_t                                current_map_id_ = -1;

  // Algorithm modules
  std::unique_ptr<algorithms::AlgorithmManager>     algorithm_manager_;
  std::shared_ptr<algorithms::RichPathfinding>      pathfinding_;
  std::shared_ptr<algorithms::RichLinkBasedPlanner> link_planner_;

  // Internal helper methods
  void buildAdjacencyList();
  void invalidateCache();
  void initializeAlgorithmModules();

  // Legacy algorithm methods (for backward compatibility)
  double calculateHeuristic(const NodeInfo &current, const NodeInfo &goal,
                            PlanningConfig::Algorithm algorithm) const;
  std::vector<int32_t>
  reconstructPath(const std::unordered_map<int32_t, int32_t> &parent,
                  int32_t                                     goal_node) const;

  // Legacy path planning methods (delegated to new modules)
  RichPathResult planDijkstra(int32_t start_node, int32_t target_node,
                              const PlanningConfig &config) const;
  RichPathResult planAStar(int32_t start_node, int32_t target_node,
                           const PlanningConfig &config) const;
  RichPathResult planDirectPath(const Eigen::Vector3d &start_pose,
                                int32_t                target_node,
                                const PlanningConfig  &config) const;
  RichPathResult planLinkBased(const Eigen::Vector3d &start_pose,
                               int32_t                target_node,
                               const PlanningConfig  &config) const;

public:
  RichGraph() = default;
  explicit RichGraph(const std::string &connection_info);

  // Database operations
  bool loadFromDatabase(const std::string &map_name);
  void clear();

  // Graph state queries
  bool   isLoaded() const { return !nodes_.empty() && !links_.empty(); }
  size_t getNodeCount() const { return nodes_.size(); }
  size_t getLinkCount() const { return links_.size(); }
  const std::string &getCurrentMapName() const { return current_map_name_; }
  int32_t            getCurrentMapId() const { return current_map_id_; }

  // Node operations
  bool                    hasNode(int32_t node_id) const override;
  const NodeInfo         &getNode(int32_t node_id) const override;
  std::optional<NodeInfo> getNodeSafe(int32_t node_id) const;
  std::vector<int32_t>    getAllNodeIds() const override;

  // Link operations
  bool                    hasLink(int32_t link_id) const override;
  const LinkInfo         &getLink(int32_t link_id) const override;
  std::optional<LinkInfo> getLinkSafe(int32_t link_id) const;
  std::vector<int32_t>    getAllLinkIds() const;

  // Graph topology queries
  std::vector<int32_t> getNodeNeighborLinks(int32_t node_id) const override;
  std::vector<int32_t> getNodeIncomingLinks(int32_t node_id) const;
  std::vector<int32_t> getNodeOutgoingLinks(int32_t node_id) const;
  std::vector<int32_t> getConnectedNodes(int32_t node_id) const;
  bool areNodesConnected(int32_t node1_id, int32_t node2_id) const;
  std::optional<int32_t> getLinkBetweenNodes(int32_t start_node,
                                             int32_t end_node) const;

  // Geometric queries
  int32_t findClosestNode(const Eigen::Vector3d &pose) const override;
  std::vector<int32_t> findClosestNodes(const Eigen::Vector3d &pose,
                                        size_t                 max_nodes) const;
  int32_t              findClosestLink(const Eigen::Vector3d &pose) const;
  std::vector<int32_t> findClosestLinks(const Eigen::Vector3d &pose,
                                        size_t                 max_links) const;
  std::vector<int32_t> findNodesInRadius(const Eigen::Vector3d &center,
                                         double radius) const override;

  // Distance calculations
  double getNodeDistance(int32_t node1_id, int32_t node2_id) const;
  double getLinkLength(int32_t link_id) const;
  double getDistanceToNode(const Eigen::Vector3d &pose, int32_t node_id) const;
  double getDistanceToLink(const Eigen::Vector3d &pose, int32_t link_id) const;

  // Path planning (legacy interface)
  RichPathResult
  planPath(const Eigen::Vector3d &start_pose, int32_t target_node_id,
           const PlanningConfig &config = PlanningConfig{}) const;
  RichPathResult
  planPath(int32_t start_node_id, int32_t target_node_id,
           const PlanningConfig &config = PlanningConfig{}) const;
  RichPathResult
  planPath(const Eigen::Vector3d &start_pose,
           const Eigen::Vector3d &target_pose,
           const PlanningConfig  &config = PlanningConfig{}) const;

  // New algorithm interface methods
  RichPathResult
  planPathWithAlgorithm(const std::string                 &algorithm_name,
                        const Eigen::Vector3d             &start_pose,
                        int32_t                            target_node_id,
                        const algorithms::AlgorithmConfig &config =
                            algorithms::AlgorithmConfig{}) const;
  RichPathResult
  planPathWithAlgorithm(const std::string &algorithm_name,
                        int32_t start_node_id, int32_t target_node_id,
                        const algorithms::AlgorithmConfig &config =
                            algorithms::AlgorithmConfig{}) const;
  RichPathResult
  planPathWithAlgorithm(const std::string                 &algorithm_name,
                        const Eigen::Vector3d             &start_pose,
                        const Eigen::Vector3d             &target_pose,
                        const algorithms::AlgorithmConfig &config =
                            algorithms::AlgorithmConfig{}) const;

  // Algorithm management
  std::vector<std::string> getAvailableAlgorithms() const;
  std::string              getRecommendedAlgorithm(size_t graph_size = 0) const;
  algorithms::AlgorithmManager &getAlgorithmManager() const;

  // Path validation
  bool   isPathValid(const std::vector<int32_t> &node_sequence) const;
  bool   isPathValid(const RichPathResult &path_result) const;
  double calculatePathLength(const std::vector<int32_t> &node_sequence) const;

  // Statistics and debugging
  struct GraphStatistics {
    size_t                    node_count;
    size_t                    link_count;
    size_t                    connected_components;
    double                    average_node_degree;
    double                    graph_density;
    std::pair<double, double> bounding_box_min;
    std::pair<double, double> bounding_box_max;
  };
  GraphStatistics getStatistics() const;

  // Validation and health checks
  struct ValidationResult {
    bool                     is_valid;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
  };
  ValidationResult validateGraph() const;

  // Export/Import for debugging
  bool exportToJson(const std::string &filepath) const;
  bool importFromJson(const std::string &filepath);

  // GraphDataInterface implementation
};

// Helper functions for pose conversions
Eigen::Vector3d poseToEigen(double x, double y, double theta);
std::tuple<double, double, double> eigenToPose(const Eigen::Vector3d &pose);

} // namespace vrobot_route_follow