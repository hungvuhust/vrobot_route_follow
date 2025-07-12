#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "vrobot_route_follow/algorithms/rich_pathfinding.hpp"
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
 * @brief Algorithm types enumeration
 */
enum class AlgorithmType {
  DIJKSTRA,
  ASTAR,
  DIRECT_PATH,
  LINK_BASED,
  HYBRID,
  CUSTOM
};

/**
 * @brief Algorithm capabilities
 */
struct AlgorithmCapabilities {
  bool supports_node_to_node       = false;
  bool supports_pose_to_pose       = false;
  bool supports_pose_to_node       = false;
  bool supports_node_to_pose       = false;
  bool supports_dynamic_obstacles  = false;
  bool supports_custom_constraints = false;
  bool supports_real_time          = false;
  bool supports_multi_goal         = false;

  // Performance characteristics
  std::optional<double> typical_computation_time_ms;
  std::optional<size_t> max_recommended_graph_size;
  std::optional<double> typical_memory_usage_mb;
};

/**
 * @brief Universal algorithm configuration
 */
struct AlgorithmConfig {
  // Algorithm selection
  AlgorithmType algorithm_type = AlgorithmType::DIJKSTRA;
  std::string   custom_algorithm_name;

  // Common parameters
  double                max_connection_distance = 5.0;
  double                goal_tolerance          = 0.5;
  std::optional<double> max_search_distance;
  std::optional<size_t> max_nodes_explored;
  std::optional<double> max_computation_time_ms;

  // Path quality preferences
  bool   prefer_shorter_paths = true;
  bool   optimize_smoothness  = false;
  double smoothness_weight    = 0.3;

  // Algorithm-specific parameters (stored as key-value pairs)
  std::unordered_map<std::string, double>      double_params;
  std::unordered_map<std::string, int>         int_params;
  std::unordered_map<std::string, bool>        bool_params;
  std::unordered_map<std::string, std::string> string_params;

  // Custom scoring functions
  std::function<double(const LinkInfo &, const NodeInfo &, const NodeInfo &)>
                                                            custom_link_scorer;
  std::function<double(const NodeInfo &, const NodeInfo &)> custom_heuristic;

  // Convenience methods for parameter access
  void setParam(const std::string &key, double value) {
    double_params[key] = value;
  }
  void setParam(const std::string &key, int value) { int_params[key] = value; }
  void setParam(const std::string &key, bool value) {
    bool_params[key] = value;
  }
  void setParam(const std::string &key, const std::string &value) {
    string_params[key] = value;
  }

  std::optional<double> getDoubleParam(const std::string &key) const {
    auto it = double_params.find(key);
    return (it != double_params.end()) ? std::optional<double>(it->second)
                                       : std::nullopt;
  }

  std::optional<int> getIntParam(const std::string &key) const {
    auto it = int_params.find(key);
    return (it != int_params.end()) ? std::optional<int>(it->second)
                                    : std::nullopt;
  }

  std::optional<bool> getBoolParam(const std::string &key) const {
    auto it = bool_params.find(key);
    return (it != bool_params.end()) ? std::optional<bool>(it->second)
                                     : std::nullopt;
  }

  std::optional<std::string> getStringParam(const std::string &key) const {
    auto it = string_params.find(key);
    return (it != string_params.end()) ? std::optional<std::string>(it->second)
                                       : std::nullopt;
  }
};

/**
 * @brief Algorithm statistics
 */
struct AlgorithmStats {
  size_t      nodes_explored          = 0;
  size_t      links_evaluated         = 0;
  double      computation_time_ms     = 0.0;
  double      memory_usage_mb         = 0.0;
  bool        search_terminated_early = false;
  std::string termination_reason;
  std::string algorithm_variant;

  // Algorithm-specific metrics
  std::unordered_map<std::string, double> custom_metrics;
};

/**
 * @brief Base interface for all path planning algorithms
 */
class PathPlanningAlgorithm {
public:
  virtual ~PathPlanningAlgorithm() = default;

  /**
   * @brief Get algorithm name
   * @return Algorithm name
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Get algorithm capabilities
   * @return Algorithm capabilities
   */
  virtual AlgorithmCapabilities getCapabilities() const = 0;

  /**
   * @brief Plan path between two nodes
   * @param start_node Starting node ID
   * @param target_node Target node ID
   * @param config Algorithm configuration
   * @return Rich path result
   */
  virtual RichPathResult
  planPath(int32_t start_node, int32_t target_node,
           const AlgorithmConfig &config = AlgorithmConfig{}) = 0;

  /**
   * @brief Plan path between two poses
   * @param start_pose Starting pose
   * @param target_pose Target pose
   * @param config Algorithm configuration
   * @return Rich path result
   */
  virtual RichPathResult
  planPath(const Eigen::Vector3d &start_pose,
           const Eigen::Vector3d &target_pose,
           const AlgorithmConfig &config = AlgorithmConfig{}) = 0;

  /**
   * @brief Plan path from pose to node
   * @param start_pose Starting pose
   * @param target_node Target node ID
   * @param config Algorithm configuration
   * @return Rich path result
   */
  virtual RichPathResult
  planPath(const Eigen::Vector3d &start_pose, int32_t target_node,
           const AlgorithmConfig &config = AlgorithmConfig{}) = 0;

  /**
   * @brief Plan path from node to pose
   * @param start_node Starting node ID
   * @param target_pose Target pose
   * @param config Algorithm configuration
   * @return Rich path result
   */
  virtual RichPathResult
  planPath(int32_t start_node, const Eigen::Vector3d &target_pose,
           const AlgorithmConfig &config = AlgorithmConfig{}) = 0;

  /**
   * @brief Get last operation statistics
   * @return Algorithm statistics
   */
  virtual AlgorithmStats getLastStats() const = 0;

  /**
   * @brief Validate configuration for this algorithm
   * @param config Configuration to validate
   * @return True if configuration is valid
   */
  virtual bool validateConfig(const AlgorithmConfig &config) const = 0;

  /**
   * @brief Get default configuration for this algorithm
   * @return Default configuration
   */
  virtual AlgorithmConfig getDefaultConfig() const = 0;
};

/**
 * @brief Factory function type for creating algorithm instances
 */
using AlgorithmFactory = std::function<std::unique_ptr<PathPlanningAlgorithm>(
    std::shared_ptr<GraphDataInterface>)>;

/**
 * @brief Algorithm registry for plugin management
 */
class AlgorithmRegistry {
private:
  std::unordered_map<std::string, AlgorithmFactory>      factories_;
  std::unordered_map<std::string, AlgorithmCapabilities> capabilities_;
  std::unordered_map<AlgorithmType, std::string>         type_to_name_;

  static std::unique_ptr<AlgorithmRegistry> instance_;

public:
  /**
   * @brief Get singleton instance
   * @return Registry instance
   */
  static AlgorithmRegistry &getInstance();

  /**
   * @brief Register an algorithm
   * @param name Algorithm name
   * @param type Algorithm type
   * @param factory Factory function
   * @param capabilities Algorithm capabilities
   */
  void registerAlgorithm(const std::string &name, AlgorithmType type,
                         AlgorithmFactory             factory,
                         const AlgorithmCapabilities &capabilities);

  /**
   * @brief Create algorithm instance by name
   * @param name Algorithm name
   * @param graph_data Graph data interface
   * @return Algorithm instance
   */
  std::unique_ptr<PathPlanningAlgorithm>
  createAlgorithm(const std::string                  &name,
                  std::shared_ptr<GraphDataInterface> graph_data);

  /**
   * @brief Create algorithm instance by type
   * @param type Algorithm type
   * @param graph_data Graph data interface
   * @return Algorithm instance
   */
  std::unique_ptr<PathPlanningAlgorithm>
  createAlgorithm(AlgorithmType                       type,
                  std::shared_ptr<GraphDataInterface> graph_data);

  /**
   * @brief Get all registered algorithm names
   * @return Vector of algorithm names
   */
  std::vector<std::string> getAvailableAlgorithms() const;

  /**
   * @brief Get algorithm capabilities by name
   * @param name Algorithm name
   * @return Algorithm capabilities
   */
  std::optional<AlgorithmCapabilities>
  getCapabilities(const std::string &name) const;

  /**
   * @brief Check if algorithm is registered
   * @param name Algorithm name
   * @return True if registered
   */
  bool isRegistered(const std::string &name) const;

  /**
   * @brief Unregister algorithm
   * @param name Algorithm name
   */
  void unregisterAlgorithm(const std::string &name);

  /**
   * @brief Clear all registered algorithms
   */
  void clear();
};

/**
 * @brief Unified algorithm manager
 */
class AlgorithmManager {
private:
  std::shared_ptr<GraphDataInterface> graph_data_;
  AlgorithmRegistry                  &registry_;

public:
  /**
   * @brief Constructor
   * @param graph_data Graph data interface
   */
  explicit AlgorithmManager(std::shared_ptr<GraphDataInterface> graph_data);

  /**
   * @brief Plan path with automatic algorithm selection
   * @param start_node Starting node ID
   * @param target_node Target node ID
   * @param config Algorithm configuration
   * @return Rich path result
   */
  RichPathResult planPath(int32_t start_node, int32_t target_node,
                          const AlgorithmConfig &config = AlgorithmConfig{});

  /**
   * @brief Plan path with automatic algorithm selection
   * @param start_pose Starting pose
   * @param target_pose Target pose
   * @param config Algorithm configuration
   * @return Rich path result
   */
  RichPathResult planPath(const Eigen::Vector3d &start_pose,
                          const Eigen::Vector3d &target_pose,
                          const AlgorithmConfig &config = AlgorithmConfig{});

  /**
   * @brief Plan path with automatic algorithm selection
   * @param start_pose Starting pose
   * @param target_node Target node ID
   * @param config Algorithm configuration
   * @return Rich path result
   */
  RichPathResult planPath(const Eigen::Vector3d &start_pose,
                          int32_t                target_node,
                          const AlgorithmConfig &config = AlgorithmConfig{});

  /**
   * @brief Plan path with automatic algorithm selection
   * @param start_node Starting node ID
   * @param target_pose Target pose
   * @param config Algorithm configuration
   * @return Rich path result
   */
  RichPathResult planPath(int32_t                start_node,
                          const Eigen::Vector3d &target_pose,
                          const AlgorithmConfig &config = AlgorithmConfig{});

  /**
   * @brief Plan path with specific algorithm
   * @param algorithm_name Algorithm name
   * @param start_node Starting node ID
   * @param target_node Target node ID
   * @param config Algorithm configuration
   * @return Rich path result
   */
  RichPathResult
  planPathWithAlgorithm(const std::string &algorithm_name, int32_t start_node,
                        int32_t                target_node,
                        const AlgorithmConfig &config = AlgorithmConfig{});

  /**
   * @brief Get suitable algorithms for given requirements
   * @param requirements Required capabilities
   * @return Vector of suitable algorithm names
   */
  std::vector<std::string>
  getSuitableAlgorithms(const AlgorithmCapabilities &requirements) const;

  /**
   * @brief Get algorithm recommendation based on graph size and requirements
   * @param graph_size Approximate graph size
   * @param requirements Required capabilities
   * @return Recommended algorithm name
   */
  std::string
  getRecommendedAlgorithm(size_t                       graph_size,
                          const AlgorithmCapabilities &requirements =
                              AlgorithmCapabilities{}) const;

  /**
   * @brief Get last operation statistics
   * @return Algorithm statistics
   */
  AlgorithmStats getLastStats() const { return last_stats_; }

private:
  mutable AlgorithmStats last_stats_;

  // Helper methods
  std::unique_ptr<PathPlanningAlgorithm>
  selectAlgorithm(const AlgorithmConfig &config);
  bool
  algorithmSupportsOperation(const std::string           &algorithm_name,
                             const AlgorithmCapabilities &requirements) const;
};

/**
 * @brief Utility functions for algorithm management
 */
namespace algorithm_utils {

/**
 * @brief Convert algorithm type to string
 * @param type Algorithm type
 * @return String representation
 */
std::string algorithmTypeToString(AlgorithmType type);

/**
 * @brief Convert string to algorithm type
 * @param str String representation
 * @return Algorithm type
 */
AlgorithmType stringToAlgorithmType(const std::string &str);

/**
 * @brief Create default configuration for algorithm type
 * @param type Algorithm type
 * @return Default configuration
 */
AlgorithmConfig createDefaultConfig(AlgorithmType type);

/**
 * @brief Merge two configurations (second overrides first)
 * @param base Base configuration
 * @param override Override configuration
 * @return Merged configuration
 */
AlgorithmConfig mergeConfigs(const AlgorithmConfig &base,
                             const AlgorithmConfig &override);

/**
 * @brief Validate configuration completeness
 * @param config Configuration to validate
 * @return True if configuration is complete
 */
bool validateConfigCompleteness(const AlgorithmConfig &config);

} // namespace algorithm_utils

} // namespace algorithms
} // namespace vrobot_route_follow