#include "vrobot_route_follow/algorithms/rich_pathfinding.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace vrobot_route_follow {
namespace algorithms {

RichPathfinding::RichPathfinding(std::shared_ptr<GraphDataInterface> graph_data)
    : graph_data_(std::move(graph_data)) {
  if (!graph_data_) {
    throw std::invalid_argument("Graph data interface cannot be null");
  }
}

double RichPathfinding::calculateLinkCost(
    const LinkInfo &link, const NodeInfo &start_node, const NodeInfo &end_node,
    const PathfindingConfig &config) const {
  // Use custom scorer if provided
  if (config.custom_scorer) {
    return config.custom_scorer(link, start_node, end_node);
  }

  // Default: Euclidean distance
  Eigen::Vector3d start_pose(start_node.x, start_node.y, start_node.theta);
  Eigen::Vector3d end_pose(end_node.x, end_node.y, end_node.theta);
  double          distance = (end_pose.head<2>() - start_pose.head<2>()).norm();

  // Apply turn penalty if configured
  if (config.avoid_sharp_turns) {
    double angle_diff = std::abs(end_pose(2) - start_pose(2));
    // Normalize angle difference to [0, π]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff > M_PI)
      angle_diff = 2 * M_PI - angle_diff;

    double turn_penalty = config.turn_penalty_factor * angle_diff;
    distance += turn_penalty;
  }

  // Apply velocity constraints if specified
  if (config.max_velocity.has_value() &&
      link.max_velocity > config.max_velocity.value()) {
    distance *= 1.2; // Penalty for exceeding max velocity
  }

  return distance;
}

double
RichPathfinding::calculateHeuristic(const NodeInfo          &current,
                                    const NodeInfo          &target,
                                    const PathfindingConfig &config) const {
  if (!config.use_euclidean_heuristic) {
    return 0.0; // Fallback to Dijkstra
  }

  Eigen::Vector3d current_pose(current.x, current.y, current.theta);
  Eigen::Vector3d target_pose(target.x, target.y, target.theta);

  double distance = (target_pose.head<2>() - current_pose.head<2>()).norm();
  return config.heuristic_weight * distance;
}

RichPathResult RichPathfinding::reconstructPath(
    const std::unordered_map<int32_t, PathfindingNode> &came_from,
    int32_t start_node, int32_t target_node,
    const std::string &algorithm_name) const {

  RichPathResult result;
  result.success       = true;
  result.algorithmUsed = algorithm_name;
  result.totalDistance = 0.0;

  // Reconstruct path by following parent pointers
  std::vector<int32_t> node_path;
  std::vector<int32_t> link_path;

  int32_t current_node = target_node;
  while (current_node != start_node) {
    node_path.push_back(current_node);

    auto it = came_from.find(current_node);
    if (it == came_from.end()) {
      result.success = false;
      result.errorMessage =
          "Path reconstruction failed: missing parent for node " +
          std::to_string(current_node);
      return result;
    }

    const PathfindingNode &path_node = it->second;
    if (path_node.parent_link_id != -1) {
      link_path.push_back(path_node.parent_link_id);
    }

    current_node = path_node.parent_node_id;
  }
  node_path.push_back(start_node);

  // Reverse to get correct order
  std::reverse(node_path.begin(), node_path.end());
  std::reverse(link_path.begin(), link_path.end());

  // Convert to NodeInfo and LinkInfo
  for (int32_t node_id : node_path) {
    if (graph_data_->hasNode(node_id)) {
      result.nodeSequence.push_back(graph_data_->getNode(node_id));
    }
  }

  for (int32_t link_id : link_path) {
    if (graph_data_->hasLink(link_id)) {
      result.linkSequence.push_back(graph_data_->getLink(link_id));
    }
  }

  // Generate pose sequence
  for (const auto &node : result.nodeSequence) {
    Eigen::Vector3d pose(node.x, node.y, node.theta);
    result.poseSequence.push_back(pose);
  }

  // Calculate total distance
  for (size_t i = 1; i < result.poseSequence.size(); ++i) {
    result.totalDistance += (result.poseSequence[i].head<2>() -
                             result.poseSequence[i - 1].head<2>())
                                .norm();
  }

  return result;
}

bool RichPathfinding::validateNodes(int32_t start_node,
                                    int32_t target_node) const {
  if (!graph_data_->hasNode(start_node)) {
    std::cerr << "Start node " << start_node << " not found in graph"
              << std::endl;
    return false;
  }

  if (!graph_data_->hasNode(target_node)) {
    std::cerr << "Target node " << target_node << " not found in graph"
              << std::endl;
    return false;
  }

  return true;
}

bool RichPathfinding::shouldTerminateSearch(const PathfindingNode   &current,
                                            const PathfindingConfig &config,
                                            size_t nodes_explored) const {
  // Check node exploration limit
  if (config.max_nodes_explored.has_value() &&
      nodes_explored >= config.max_nodes_explored.value()) {
    return true;
  }

  // Check search distance limit
  if (config.max_search_distance.has_value() &&
      current.cost >= config.max_search_distance.value()) {
    return true;
  }

  return false;
}

RichPathResult
RichPathfinding::planDijkstra(int32_t start_node, int32_t target_node,
                              const PathfindingConfig &config) const {
  auto start_time = std::chrono::high_resolution_clock::now();
  last_stats_     = PathfindingStats{};

  RichPathResult result;
  result.algorithmUsed = "Dijkstra";
  result.success       = false;

  // Validate input
  if (!validateNodes(start_node, target_node)) {
    result.errorMessage = "Invalid start or target node";
    return result;
  }

  // Priority queue: min-heap based on cost
  std::priority_queue<PathfindingNode, std::vector<PathfindingNode>,
                      std::greater<PathfindingNode>>
                                               open_set;
  std::unordered_map<int32_t, PathfindingNode> came_from;
  std::unordered_set<int32_t>                  closed_set;

  // Initialize with start node
  PathfindingNode start_path_node(start_node, 0.0);
  open_set.push(start_path_node);
  came_from[start_node] = start_path_node;

  size_t nodes_explored = 0;

  while (!open_set.empty()) {
    PathfindingNode current = open_set.top();
    open_set.pop();

    // Skip if already processed
    if (closed_set.count(current.node_id)) {
      continue;
    }

    closed_set.insert(current.node_id);
    nodes_explored++;

    // Check termination conditions
    if (shouldTerminateSearch(current, config, nodes_explored)) {
      last_stats_.search_terminated_early = true;
      last_stats_.termination_reason      = "Search limits exceeded";
      break;
    }

    // Goal reached
    if (current.node_id == target_node) {
      result = reconstructPath(came_from, start_node, target_node, "Dijkstra");
      break;
    }

    // Explore neighbors
    auto neighbor_links = graph_data_->getNodeNeighborLinks(current.node_id);
    for (int32_t link_id : neighbor_links) {
      if (!graph_data_->hasLink(link_id))
        continue;

      const LinkInfo &link = graph_data_->getLink(link_id);
      int32_t         neighbor_id =
          (link.id_start == current.node_id) ? link.id_end : link.id_start;

      // Skip if already processed
      if (closed_set.count(neighbor_id)) {
        continue;
      }

      // Calculate cost to neighbor
      const NodeInfo &current_node  = graph_data_->getNode(current.node_id);
      const NodeInfo &neighbor_node = graph_data_->getNode(neighbor_id);
      double          link_cost =
          calculateLinkCost(link, current_node, neighbor_node, config);
      double new_cost = current.cost + link_cost;

      // Check if this path is better
      auto existing = came_from.find(neighbor_id);
      if (existing == came_from.end() || new_cost < existing->second.cost) {
        PathfindingNode neighbor_path_node(neighbor_id, new_cost, 0.0,
                                           current.node_id, link_id);
        came_from[neighbor_id] = neighbor_path_node;
        open_set.push(neighbor_path_node);
      }

      last_stats_.links_evaluated++;
    }
  }

  // Update statistics
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  last_stats_.computation_time_ms = duration.count() / 1000.0;
  last_stats_.nodes_explored      = nodes_explored;

  if (!result.success && result.errorMessage.empty()) {
    result.errorMessage = "No path found between nodes " +
                          std::to_string(start_node) + " and " +
                          std::to_string(target_node);
  }

  return result;
}

RichPathResult
RichPathfinding::planAStar(int32_t start_node, int32_t target_node,
                           const PathfindingConfig &config) const {
  auto start_time = std::chrono::high_resolution_clock::now();
  last_stats_     = PathfindingStats{};

  RichPathResult result;
  result.algorithmUsed = "A*";
  result.success       = false;

  // Validate input
  if (!validateNodes(start_node, target_node)) {
    result.errorMessage = "Invalid start or target node";
    return result;
  }

  const NodeInfo &target_node_info = graph_data_->getNode(target_node);

  // Priority queue: min-heap based on f = g + h
  std::priority_queue<PathfindingNode, std::vector<PathfindingNode>,
                      std::greater<PathfindingNode>>
                                               open_set;
  std::unordered_map<int32_t, PathfindingNode> came_from;
  std::unordered_set<int32_t>                  closed_set;

  // Initialize with start node
  const NodeInfo &start_node_info = graph_data_->getNode(start_node);
  double          initial_heuristic =
      calculateHeuristic(start_node_info, target_node_info, config);
  PathfindingNode start_path_node(start_node, 0.0, initial_heuristic);
  open_set.push(start_path_node);
  came_from[start_node] = start_path_node;

  size_t nodes_explored = 0;

  while (!open_set.empty()) {
    PathfindingNode current = open_set.top();
    open_set.pop();

    // Skip if already processed
    if (closed_set.count(current.node_id)) {
      continue;
    }

    closed_set.insert(current.node_id);
    nodes_explored++;

    // Check termination conditions
    if (shouldTerminateSearch(current, config, nodes_explored)) {
      last_stats_.search_terminated_early = true;
      last_stats_.termination_reason      = "Search limits exceeded";
      break;
    }

    // Goal reached
    if (current.node_id == target_node) {
      result = reconstructPath(came_from, start_node, target_node, "A*");
      break;
    }

    // Explore neighbors
    auto neighbor_links = graph_data_->getNodeNeighborLinks(current.node_id);
    for (int32_t link_id : neighbor_links) {
      if (!graph_data_->hasLink(link_id))
        continue;

      const LinkInfo &link = graph_data_->getLink(link_id);
      int32_t         neighbor_id =
          (link.id_start == current.node_id) ? link.id_end : link.id_start;

      // Skip if already processed
      if (closed_set.count(neighbor_id)) {
        continue;
      }

      // Calculate cost to neighbor
      const NodeInfo &current_node  = graph_data_->getNode(current.node_id);
      const NodeInfo &neighbor_node = graph_data_->getNode(neighbor_id);
      double          link_cost =
          calculateLinkCost(link, current_node, neighbor_node, config);
      double new_cost = current.cost + link_cost;

      // Calculate heuristic for neighbor
      double heuristic =
          calculateHeuristic(neighbor_node, target_node_info, config);

      // Check if this path is better
      auto existing = came_from.find(neighbor_id);
      if (existing == came_from.end() || new_cost < existing->second.cost) {
        PathfindingNode neighbor_path_node(neighbor_id, new_cost, heuristic,
                                           current.node_id, link_id);
        came_from[neighbor_id] = neighbor_path_node;
        open_set.push(neighbor_path_node);
      }

      last_stats_.links_evaluated++;
    }
  }

  // Update statistics
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  last_stats_.computation_time_ms = duration.count() / 1000.0;
  last_stats_.nodes_explored      = nodes_explored;

  if (!result.success && result.errorMessage.empty()) {
    result.errorMessage = "No path found between nodes " +
                          std::to_string(start_node) + " and " +
                          std::to_string(target_node);
  }

  return result;
}

RichPathResult
RichPathfinding::planDirectPath(const Eigen::Vector3d   &start_pose,
                                int32_t                  target_node,
                                const PathfindingConfig &config) const {
  auto start_time = std::chrono::high_resolution_clock::now();
  last_stats_     = PathfindingStats{};

  RichPathResult result;
  result.algorithmUsed = "DirectPath";
  result.success       = false;

  // Validate target node
  if (!graph_data_->hasNode(target_node)) {
    result.errorMessage =
        "Target node " + std::to_string(target_node) + " not found";
    return result;
  }

  const NodeInfo &target_node_info = graph_data_->getNode(target_node);
  Eigen::Vector3d target_pose(target_node_info.x, target_node_info.y,
                              target_node_info.theta);

  // Check if direct path is within connection distance
  double distance = (target_pose.head<2>() - start_pose.head<2>()).norm();
  if (distance > config.max_connection_distance) {
    result.errorMessage =
        "Target too far for direct connection: " + std::to_string(distance) +
        " > " + std::to_string(config.max_connection_distance);
    return result;
  }

  // Create direct path result
  result.success       = true;
  result.totalDistance = distance;

  // Add start and target poses
  result.poseSequence.push_back(start_pose);
  result.poseSequence.push_back(target_pose);

  // Add target node
  result.nodeSequence.push_back(target_node_info);

  // Update statistics
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  last_stats_.computation_time_ms = duration.count() / 1000.0;
  last_stats_.nodes_explored      = 1;

  return result;
}

// Factory methods
std::unique_ptr<RichPathfinding>
PathfindingFactory::create(std::shared_ptr<GraphDataInterface> graph_data) {
  return std::make_unique<RichPathfinding>(graph_data);
}

PathfindingConfig PathfindingFactory::createDefaultConfig() {
  return PathfindingConfig{};
}

PathfindingConfig PathfindingFactory::createSpeedOptimizedConfig() {
  PathfindingConfig config;
  config.max_nodes_explored   = 1000;
  config.max_search_distance  = 50.0;
  config.prefer_shorter_paths = true;
  config.avoid_sharp_turns    = false;
  config.heuristic_weight     = 1.5; // More aggressive heuristic
  return config;
}

PathfindingConfig PathfindingFactory::createAccuracyOptimizedConfig() {
  PathfindingConfig config;
  config.max_nodes_explored   = std::nullopt; // No limit
  config.max_search_distance  = std::nullopt; // No limit
  config.prefer_shorter_paths = true;
  config.avoid_sharp_turns    = true;
  config.turn_penalty_factor  = 2.0;
  config.heuristic_weight     = 1.0; // Conservative heuristic
  return config;
}

} // namespace algorithms
} // namespace vrobot_route_follow