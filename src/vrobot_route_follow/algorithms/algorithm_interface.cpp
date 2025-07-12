#include "vrobot_route_follow/algorithms/algorithm_interface.hpp"
#include <algorithm>
#include <stdexcept>

namespace vrobot_route_follow {
namespace algorithms {

// Static member definition
std::unique_ptr<AlgorithmRegistry> AlgorithmRegistry::instance_;

AlgorithmRegistry &AlgorithmRegistry::getInstance() {
  if (!instance_) {
    instance_ = std::make_unique<AlgorithmRegistry>();
  }
  return *instance_;
}

void AlgorithmRegistry::registerAlgorithm(
    const std::string &name, AlgorithmType type, AlgorithmFactory factory,
    const AlgorithmCapabilities &capabilities) {
  factories_[name]    = factory;
  capabilities_[name] = capabilities;
  type_to_name_[type] = name;
}

std::unique_ptr<PathPlanningAlgorithm> AlgorithmRegistry::createAlgorithm(
    const std::string &name, std::shared_ptr<GraphDataInterface> graph_data) {
  auto it = factories_.find(name);
  if (it == factories_.end()) {
    throw std::runtime_error("Algorithm not found: " + name);
  }
  return it->second(graph_data);
}

std::unique_ptr<PathPlanningAlgorithm> AlgorithmRegistry::createAlgorithm(
    AlgorithmType type, std::shared_ptr<GraphDataInterface> graph_data) {
  auto it = type_to_name_.find(type);
  if (it == type_to_name_.end()) {
    throw std::runtime_error("Algorithm type not registered: " +
                             std::to_string(static_cast<int>(type)));
  }
  return createAlgorithm(it->second, graph_data);
}

std::vector<std::string> AlgorithmRegistry::getAvailableAlgorithms() const {
  std::vector<std::string> algorithms;
  for (const auto &pair : factories_) {
    algorithms.push_back(pair.first);
  }
  return algorithms;
}

std::optional<AlgorithmCapabilities>
AlgorithmRegistry::getCapabilities(const std::string &name) const {
  auto it = capabilities_.find(name);
  if (it != capabilities_.end()) {
    return it->second;
  }
  return std::nullopt;
}

bool AlgorithmRegistry::isRegistered(const std::string &name) const {
  return factories_.find(name) != factories_.end();
}

void AlgorithmRegistry::unregisterAlgorithm(const std::string &name) {
  factories_.erase(name);
  capabilities_.erase(name);

  // Remove from type mapping
  for (auto it = type_to_name_.begin(); it != type_to_name_.end(); ++it) {
    if (it->second == name) {
      type_to_name_.erase(it);
      break;
    }
  }
}

void AlgorithmRegistry::clear() {
  factories_.clear();
  capabilities_.clear();
  type_to_name_.clear();
}

// AlgorithmManager implementation
AlgorithmManager::AlgorithmManager(
    std::shared_ptr<GraphDataInterface> graph_data)
    : graph_data_(std::move(graph_data)),
      registry_(AlgorithmRegistry::getInstance()) {
  if (!graph_data_) {
    throw std::invalid_argument("Graph data interface cannot be null");
  }
}

RichPathResult AlgorithmManager::planPath(int32_t                start_node,
                                          int32_t                target_node,
                                          const AlgorithmConfig &config) {
  auto algorithm = selectAlgorithm(config);
  if (!algorithm) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "No suitable algorithm found";
    return result;
  }

  auto result = algorithm->planPath(start_node, target_node, config);
  last_stats_ = algorithm->getLastStats();
  return result;
}

RichPathResult AlgorithmManager::planPath(const Eigen::Vector3d &start_pose,
                                          const Eigen::Vector3d &target_pose,
                                          const AlgorithmConfig &config) {
  auto algorithm = selectAlgorithm(config);
  if (!algorithm) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "No suitable algorithm found";
    return result;
  }

  auto result = algorithm->planPath(start_pose, target_pose, config);
  last_stats_ = algorithm->getLastStats();
  return result;
}

RichPathResult AlgorithmManager::planPath(const Eigen::Vector3d &start_pose,
                                          int32_t                target_node,
                                          const AlgorithmConfig &config) {
  auto algorithm = selectAlgorithm(config);
  if (!algorithm) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "No suitable algorithm found";
    return result;
  }

  auto result = algorithm->planPath(start_pose, target_node, config);
  last_stats_ = algorithm->getLastStats();
  return result;
}

RichPathResult AlgorithmManager::planPath(int32_t                start_node,
                                          const Eigen::Vector3d &target_pose,
                                          const AlgorithmConfig &config) {
  auto algorithm = selectAlgorithm(config);
  if (!algorithm) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "No suitable algorithm found";
    return result;
  }

  auto result = algorithm->planPath(start_node, target_pose, config);
  last_stats_ = algorithm->getLastStats();
  return result;
}

RichPathResult
AlgorithmManager::planPathWithAlgorithm(const std::string &algorithm_name,
                                        int32_t start_node, int32_t target_node,
                                        const AlgorithmConfig &config) {
  auto algorithm = registry_.createAlgorithm(algorithm_name, graph_data_);
  if (!algorithm) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "Algorithm not found: " + algorithm_name;
    return result;
  }

  auto result = algorithm->planPath(start_node, target_node, config);
  last_stats_ = algorithm->getLastStats();
  return result;
}

std::vector<std::string> AlgorithmManager::getSuitableAlgorithms(
    const AlgorithmCapabilities &requirements) const {
  std::vector<std::string> suitable;

  for (const auto &algorithm_name : registry_.getAvailableAlgorithms()) {
    if (algorithmSupportsOperation(algorithm_name, requirements)) {
      suitable.push_back(algorithm_name);
    }
  }

  return suitable;
}

std::string AlgorithmManager::getRecommendedAlgorithm(
    size_t graph_size, const AlgorithmCapabilities &requirements) const {
  auto suitable = getSuitableAlgorithms(requirements);
  if (suitable.empty()) {
    return ""; // No suitable algorithm
  }

  // Simple heuristic: prefer A* for medium graphs, Dijkstra for large graphs
  if (graph_size < 1000) {
    auto it = std::find(suitable.begin(), suitable.end(), "A*");
    if (it != suitable.end())
      return "A*";
  } else {
    auto it = std::find(suitable.begin(), suitable.end(), "Dijkstra");
    if (it != suitable.end())
      return "Dijkstra";
  }

  // Return first suitable algorithm
  return suitable[0];
}

std::unique_ptr<PathPlanningAlgorithm>
AlgorithmManager::selectAlgorithm(const AlgorithmConfig &config) {
  if (!config.custom_algorithm_name.empty()) {
    return registry_.createAlgorithm(config.custom_algorithm_name, graph_data_);
  }

  return registry_.createAlgorithm(config.algorithm_type, graph_data_);
}

bool AlgorithmManager::algorithmSupportsOperation(
    const std::string           &algorithm_name,
    const AlgorithmCapabilities &requirements) const {
  auto caps = registry_.getCapabilities(algorithm_name);
  if (!caps)
    return false;

  // Check each requirement
  if (requirements.supports_node_to_node && !caps->supports_node_to_node)
    return false;
  if (requirements.supports_pose_to_pose && !caps->supports_pose_to_pose)
    return false;
  if (requirements.supports_pose_to_node && !caps->supports_pose_to_node)
    return false;
  if (requirements.supports_node_to_pose && !caps->supports_node_to_pose)
    return false;
  if (requirements.supports_dynamic_obstacles &&
      !caps->supports_dynamic_obstacles)
    return false;
  if (requirements.supports_custom_constraints &&
      !caps->supports_custom_constraints)
    return false;
  if (requirements.supports_real_time && !caps->supports_real_time)
    return false;
  if (requirements.supports_multi_goal && !caps->supports_multi_goal)
    return false;

  return true;
}

// Utility functions
namespace algorithm_utils {

std::string algorithmTypeToString(AlgorithmType type) {
  switch (type) {
  case AlgorithmType::DIJKSTRA: return "Dijkstra";
  case AlgorithmType::ASTAR: return "A*";
  case AlgorithmType::DIRECT_PATH: return "DirectPath";
  case AlgorithmType::LINK_BASED: return "LinkBased";
  case AlgorithmType::HYBRID: return "Hybrid";
  case AlgorithmType::CUSTOM: return "Custom";
  default: return "Unknown";
  }
}

AlgorithmType stringToAlgorithmType(const std::string &str) {
  if (str == "Dijkstra")
    return AlgorithmType::DIJKSTRA;
  if (str == "A*")
    return AlgorithmType::ASTAR;
  if (str == "DirectPath")
    return AlgorithmType::DIRECT_PATH;
  if (str == "LinkBased")
    return AlgorithmType::LINK_BASED;
  if (str == "Hybrid")
    return AlgorithmType::HYBRID;
  if (str == "Custom")
    return AlgorithmType::CUSTOM;

  throw std::invalid_argument("Unknown algorithm type: " + str);
}

AlgorithmConfig createDefaultConfig(AlgorithmType type) {
  AlgorithmConfig config;
  config.algorithm_type = type;

  switch (type) {
  case AlgorithmType::DIJKSTRA: config.setParam("use_heuristic", false); break;
  case AlgorithmType::ASTAR:
    config.setParam("heuristic_weight", 1.0);
    config.setParam("use_euclidean_heuristic", true);
    break;
  case AlgorithmType::DIRECT_PATH: config.max_connection_distance = 10.0; break;
  case AlgorithmType::LINK_BASED:
    config.setParam("prefer_direct_connections", true);
    config.setParam("use_fallback", true);
    break;
  default: break;
  }

  return config;
}

AlgorithmConfig mergeConfigs(const AlgorithmConfig &base,
                             const AlgorithmConfig &override) {
  AlgorithmConfig merged = base;

  // Override basic parameters
  if (override.algorithm_type != AlgorithmType::DIJKSTRA) {
    merged.algorithm_type = override.algorithm_type;
  }
  if (!override.custom_algorithm_name.empty()) {
    merged.custom_algorithm_name = override.custom_algorithm_name;
  }

  // Merge parameter maps
  for (const auto &pair : override.double_params) {
    merged.double_params[pair.first] = pair.second;
  }
  for (const auto &pair : override.int_params) {
    merged.int_params[pair.first] = pair.second;
  }
  for (const auto &pair : override.bool_params) {
    merged.bool_params[pair.first] = pair.second;
  }
  for (const auto &pair : override.string_params) {
    merged.string_params[pair.first] = pair.second;
  }

  // Override functions if provided
  if (override.custom_link_scorer) {
    merged.custom_link_scorer = override.custom_link_scorer;
  }
  if (override.custom_heuristic) {
    merged.custom_heuristic = override.custom_heuristic;
  }

  return merged;
}

bool validateConfigCompleteness(const AlgorithmConfig &config) {
  // Basic validation
  if (config.max_connection_distance <= 0)
    return false;
  if (config.goal_tolerance <= 0)
    return false;

  // Algorithm-specific validation
  switch (config.algorithm_type) {
  case AlgorithmType::ASTAR:
    if (auto weight = config.getDoubleParam("heuristic_weight")) {
      if (weight.value() <= 0)
        return false;
    }
    break;
  case AlgorithmType::DIRECT_PATH:
    if (config.max_connection_distance < 1.0)
      return false;
    break;
  default: break;
  }

  return true;
}

} // namespace algorithm_utils

} // namespace algorithms
} // namespace vrobot_route_follow