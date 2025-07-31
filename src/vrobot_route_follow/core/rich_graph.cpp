#include "vrobot_route_follow/core/rich_graph.hpp"
#include "vrobot_route_follow/utils/database_converter.hpp"

#include <algorithm>
#include <cmath>
#include <drogon/orm/DbClient.h>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <queue>
#include <set>

namespace vrobot_route_follow {

// Helper functions for pose conversions
Eigen::Vector3d poseToEigen(double x, double y, double theta) {
  return Eigen::Vector3d(x, y, theta);
}

std::tuple<double, double, double> eigenToPose(const Eigen::Vector3d &pose) {
  return std::make_tuple(pose.x(), pose.y(), pose.z());
}

RichGraph::RichGraph(const std::string &connection_info) {
  db_client_ = drogon::orm::DbClient::newPgClient(connection_info, 1);
}

bool RichGraph::loadFromDatabase(const std::string &map_name) {
  if (!db_client_) {
    return false;
  }
  clear();

  try {
    // Use ORM to load map
    drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Map> mapMapper(
        db_client_);
    auto mapCriteria = drogon::orm::Criteria(
        drogon_model::amr_01::amr_ros2::Map::Cols::_map_name,
        drogon::orm::CompareOperator::EQ, map_name);
    auto maps = mapMapper.findBy(mapCriteria);

    if (maps.empty()) {
      return false;
    }

    current_map_id_   = maps[0].getValueOfIdMap();
    current_map_name_ = map_name;

    // Use ORM to load nodes
    drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Node> nodeMapper(
        db_client_);
    auto nodeCriteria = drogon::orm::Criteria(
        drogon_model::amr_01::amr_ros2::Node::Cols::_map_id,
        drogon::orm::CompareOperator::EQ, current_map_id_);
    auto db_nodes = nodeMapper.findBy(nodeCriteria);

    for (const auto &db_node : db_nodes) {
      NodeInfo node =
          vrobot_route_follow::utils::DatabaseConverter::convertNode(db_node);
      nodes_[node.id] = std::move(node);
    }

    // Use ORM to load links
    drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Straightlink>
         linkMapper(db_client_);
    auto linkCriteria = drogon::orm::Criteria(
        drogon_model::amr_01::amr_ros2::Straightlink::Cols::_map_id,
        drogon::orm::CompareOperator::EQ, current_map_id_);
    auto db_links = linkMapper.findBy(linkCriteria);

    for (const auto &db_link : db_links) {
      LinkInfo link =
          vrobot_route_follow::utils::DatabaseConverter::convertLink(db_link);
      links_[link.id_straight_link] = std::move(link);
    }

    // Build adjacency list
    buildAdjacencyList();

    return !nodes_.empty() && !links_.empty();

  } catch (const std::exception &e) {
    clear();
    return false;
  }
}

void RichGraph::clear() {
  nodes_.clear();
  links_.clear();
  adjacency_list_.clear();
  reverse_adjacency_list_.clear();
  adjacency_cache_valid_ = false;
  current_map_name_.clear();
  current_map_id_ = -1;
}

void RichGraph::buildAdjacencyList() {
  adjacency_list_.clear();

  // Initialize adjacency lists for all nodes
  for (const auto &[node_id, node] : nodes_) {
    adjacency_list_[node_id] = std::vector<int32_t>();
  }

  // Build adjacency list from links
  for (const auto &[link_id, link] : links_) {
    if (nodes_.count(link.id_start) && nodes_.count(link.id_end)) {
      adjacency_list_[link.id_start].push_back(link_id);
    }
  }

  invalidateCache();
}

void RichGraph::invalidateCache() {
  reverse_adjacency_list_.clear();
  adjacency_cache_valid_ = false;
}

// Node operations
bool RichGraph::hasNode(int32_t node_id) const {
  return nodes_.count(node_id) > 0;
}

const NodeInfo &RichGraph::getNode(int32_t node_id) const {
  return nodes_.at(node_id);
}

std::optional<NodeInfo> RichGraph::getNodeSafe(int32_t node_id) const {
  auto it = nodes_.find(node_id);
  if (it != nodes_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<int32_t> RichGraph::getAllNodeIds() const {
  std::vector<int32_t> node_ids;
  node_ids.reserve(nodes_.size());
  for (const auto &[node_id, node] : nodes_) {
    node_ids.push_back(node_id);
  }
  return node_ids;
}

// Link operations
bool RichGraph::hasLink(int32_t link_id) const {
  return links_.count(link_id) > 0;
}

const LinkInfo &RichGraph::getLink(int32_t link_id) const {
  return links_.at(link_id);
}

std::optional<LinkInfo> RichGraph::getLinkSafe(int32_t link_id) const {
  auto it = links_.find(link_id);
  if (it != links_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<int32_t> RichGraph::getAllLinkIds() const {
  std::vector<int32_t> link_ids;
  link_ids.reserve(links_.size());
  for (const auto &[link_id, link] : links_) {
    link_ids.push_back(link_id);
  }
  return link_ids;
}

// Graph topology queries
std::vector<int32_t> RichGraph::getNodeNeighborLinks(int32_t node_id) const {
  auto it = adjacency_list_.find(node_id);
  if (it != adjacency_list_.end()) {
    return it->second;
  }
  return {};
}

std::vector<int32_t> RichGraph::getNodeIncomingLinks(int32_t node_id) const {
  if (!adjacency_cache_valid_) {
    // Build reverse adjacency list
    reverse_adjacency_list_.clear();
    for (const auto &[start_node, link_ids] : adjacency_list_) {
      for (int32_t link_id : link_ids) {
        const auto &link = links_.at(link_id);
        reverse_adjacency_list_[link.id_end].push_back(link_id);
      }
    }
    adjacency_cache_valid_ = true;
  }

  auto it = reverse_adjacency_list_.find(node_id);
  if (it != reverse_adjacency_list_.end()) {
    return it->second;
  }
  return {};
}

std::vector<int32_t> RichGraph::getNodeOutgoingLinks(int32_t node_id) const {
  return getNodeNeighborLinks(node_id);
}

std::vector<int32_t> RichGraph::getConnectedNodes(int32_t node_id) const {
  std::vector<int32_t> connected_nodes;

  // Get outgoing connections
  for (int32_t link_id : getNodeNeighborLinks(node_id)) {
    const auto &link = links_.at(link_id);
    connected_nodes.push_back(link.id_end);
  }

  // Get incoming connections
  for (int32_t link_id : getNodeIncomingLinks(node_id)) {
    const auto &link = links_.at(link_id);
    connected_nodes.push_back(link.id_start);
  }

  // Remove duplicates
  std::sort(connected_nodes.begin(), connected_nodes.end());
  connected_nodes.erase(
      std::unique(connected_nodes.begin(), connected_nodes.end()),
      connected_nodes.end());

  return connected_nodes;
}

bool RichGraph::areNodesConnected(int32_t node1_id, int32_t node2_id) const {
  return getLinkBetweenNodes(node1_id, node2_id).has_value() ||
         getLinkBetweenNodes(node2_id, node1_id).has_value();
}

std::optional<int32_t> RichGraph::getLinkBetweenNodes(int32_t start_node,
                                                      int32_t end_node) const {
  for (int32_t link_id : getNodeNeighborLinks(start_node)) {
    const auto &link = links_.at(link_id);
    if (link.id_end == end_node) {
      return link_id;
    }
  }
  return std::nullopt;
}

// Geometric queries
int32_t RichGraph::findClosestNode(const Eigen::Vector3d &pose) const {
  if (nodes_.empty()) {
    return -1;
  }

  int32_t closest_node = -1;
  double  min_distance = std::numeric_limits<double>::max();

  for (const auto &[node_id, node] : nodes_) {
    double distance = getDistanceToNode(pose, node_id);
    if (distance < min_distance) {
      min_distance = distance;
      closest_node = node_id;
    }
  }

  return closest_node;
}

std::vector<int32_t> RichGraph::findNodesInRadius(const Eigen::Vector3d &center,
                                                  double radius) const {
  std::vector<int32_t> nodes_in_radius;

  for (const auto &[node_id, node] : nodes_) {
    double distance = getDistanceToNode(center, node_id);
    if (distance <= radius) {
      nodes_in_radius.push_back(node_id);
    }
  }

  return nodes_in_radius;
}

std::vector<int32_t> RichGraph::findClosestNodes(const Eigen::Vector3d &pose,
                                                 size_t max_nodes) const {
  std::vector<std::pair<double, int32_t>> node_distances;

  for (const auto &[node_id, node] : nodes_) {
    double distance = getDistanceToNode(pose, node_id);
    node_distances.emplace_back(distance, node_id);
  }

  std::sort(node_distances.begin(), node_distances.end());

  std::vector<int32_t> closest_nodes;
  size_t               count = std::min(max_nodes, node_distances.size());
  closest_nodes.reserve(count);

  for (size_t i = 0; i < count; ++i) {
    closest_nodes.push_back(node_distances[i].second);
  }

  return closest_nodes;
}

int32_t RichGraph::findClosestLink(const Eigen::Vector3d &pose) const {
  if (links_.empty()) {
    return -1;
  }

  int32_t closest_link = -1;
  double  min_distance = std::numeric_limits<double>::max();

  for (const auto &[link_id, link] : links_) {
    double distance = getDistanceToLink(pose, link_id);
    if (distance < min_distance) {
      min_distance = distance;
      closest_link = link_id;
    }
  }

  return closest_link;
}

std::vector<int32_t> RichGraph::findClosestLinks(const Eigen::Vector3d &pose,
                                                 size_t max_links) const {
  std::vector<std::pair<double, int32_t>> link_distances;

  for (const auto &[link_id, link] : links_) {
    double distance = getDistanceToLink(pose, link_id);
    link_distances.emplace_back(distance, link_id);
  }

  std::sort(link_distances.begin(), link_distances.end());

  std::vector<int32_t> closest_links;
  size_t               count = std::min(max_links, link_distances.size());
  closest_links.reserve(count);

  for (size_t i = 0; i < count; ++i) {
    closest_links.push_back(link_distances[i].second);
  }

  return closest_links;
}

// Distance calculations
double RichGraph::getNodeDistance(int32_t node1_id, int32_t node2_id) const {
  const auto &node1 = nodes_.at(node1_id);
  const auto &node2 = nodes_.at(node2_id);

  double dx = node2.x - node1.x;
  double dy = node2.y - node1.y;
  return std::sqrt(dx * dx + dy * dy);
}

double RichGraph::getLinkLength(int32_t link_id) const {
  const auto &link = links_.at(link_id);
  return getNodeDistance(link.id_start, link.id_end);
}

double RichGraph::getDistanceToNode(const Eigen::Vector3d &pose,
                                    int32_t                node_id) const {
  const auto &node = nodes_.at(node_id);

  double dx = node.x - pose.x();
  double dy = node.y - pose.y();
  return std::sqrt(dx * dx + dy * dy);
}

double RichGraph::getDistanceToLink(const Eigen::Vector3d &pose,
                                    int32_t                link_id) const {
  const auto &link       = links_.at(link_id);
  const auto &start_node = nodes_.at(link.id_start);
  const auto &end_node   = nodes_.at(link.id_end);

  // Point-to-line segment distance calculation
  Eigen::Vector2d p(pose.x(), pose.y());
  Eigen::Vector2d a(start_node.x, start_node.y);
  Eigen::Vector2d b(end_node.x, end_node.y);

  Eigen::Vector2d ab = b - a;
  Eigen::Vector2d ap = p - a;

  double ab_length_sq = ab.squaredNorm();
  if (ab_length_sq < 1e-10) {
    // Degenerate case: start and end are the same
    return (p - a).norm();
  }

  double t = ap.dot(ab) / ab_length_sq;
  t        = std::max(0.0, std::min(1.0, t)); // Clamp to [0, 1]

  Eigen::Vector2d projection = a + t * ab;
  return (p - projection).norm();
}

// Path planning entry points
RichPathResult RichGraph::planPath(const Eigen::Vector3d &start_pose,
                                   int32_t                target_node_id,
                                   const PlanningConfig  &config) const {
  if (!hasNode(target_node_id)) {
    RichPathResult result;
    result.success       = false;
    result.algorithmUsed = "Invalid target node";
    return result;
  }

  switch (config.algorithm) {
  case PlanningConfig::Algorithm::DIJKSTRA: {
    int32_t start_node = findClosestNode(start_pose);
    if (start_node == -1) {
      RichPathResult result;
      result.success       = false;
      result.algorithmUsed = "No start node found";
      return result;
    }
    return planDijkstra(start_node, target_node_id, config);
  }
  case PlanningConfig::Algorithm::A_STAR: {
    int32_t start_node = findClosestNode(start_pose);
    if (start_node == -1) {
      RichPathResult result;
      result.success       = false;
      result.algorithmUsed = "No start node found";
      return result;
    }
    return planAStar(start_node, target_node_id, config);
  }
  case PlanningConfig::Algorithm::DIRECT_PATH:
    return planDirectPath(start_pose, target_node_id, config);
  case PlanningConfig::Algorithm::LINK_BASED:
    return planLinkBased(start_pose, target_node_id, config);
  default: {
    RichPathResult result;
    result.success       = false;
    result.algorithmUsed = "Unknown algorithm";
    return result;
  }
  }
}

RichPathResult RichGraph::planPath(int32_t               start_node_id,
                                   int32_t               target_node_id,
                                   const PlanningConfig &config) const {
  if (!hasNode(start_node_id) || !hasNode(target_node_id)) {
    RichPathResult result;
    result.success       = false;
    result.algorithmUsed = "Invalid node IDs";
    return result;
  }

  switch (config.algorithm) {
  case PlanningConfig::Algorithm::DIJKSTRA:
    return planDijkstra(start_node_id, target_node_id, config);
  case PlanningConfig::Algorithm::A_STAR:
    return planAStar(start_node_id, target_node_id, config);
  default: {
    // For other algorithms, convert to pose-based planning
    const auto     &start_node = getNode(start_node_id);
    Eigen::Vector3d start_pose(start_node.x, start_node.y, start_node.theta);
    return planPath(start_pose, target_node_id, config);
  }
  }
}

RichPathResult RichGraph::planPath(const Eigen::Vector3d &start_pose,
                                   const Eigen::Vector3d &target_pose,
                                   const PlanningConfig  &config) const {
  int32_t target_node = findClosestNode(target_pose);
  if (target_node == -1) {
    RichPathResult result;
    result.success       = false;
    result.algorithmUsed = "No target node found";
    return result;
  }

  return planPath(start_pose, target_node, config);
}

// Algorithm implementations

double
RichGraph::calculateHeuristic(const NodeInfo &current, const NodeInfo &goal,
                              PlanningConfig::Algorithm algorithm) const {
  switch (algorithm) {
  case PlanningConfig::Algorithm::A_STAR: {
    // Euclidean distance heuristic
    double dx = goal.x - current.x;
    double dy = goal.y - current.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  default: return 0.0; // No heuristic for Dijkstra
  }
}

std::vector<int32_t>
RichGraph::reconstructPath(const std::unordered_map<int32_t, int32_t> &parent,
                           int32_t goal_node) const {
  std::vector<int32_t> path;
  int32_t              current = goal_node;

  while (parent.find(current) != parent.end()) {
    path.push_back(current);
    current = parent.at(current);
  }
  path.push_back(current); // Add start node

  std::reverse(path.begin(), path.end());
  return path;
}

RichPathResult RichGraph::planDijkstra(int32_t start_node, int32_t target_node,
                                       const PlanningConfig &config) const {
  RichPathResult result;
  result.algorithmUsed = "Dijkstra";

  if (start_node == target_node) {
    result.nodeSequence  = {getNode(start_node)};
    result.success       = true;
    result.totalDistance = 0.0;

    // Convert to pose sequence
    const auto &node    = getNode(start_node);
    result.poseSequence = {Eigen::Vector3d(node.x, node.y, node.theta)};
    return result;
  }

  // Priority queue: (distance, node_id)
  std::priority_queue<std::pair<double, int32_t>,
                      std::vector<std::pair<double, int32_t>>,
                      std::greater<std::pair<double, int32_t>>>
      pq;

  std::unordered_map<int32_t, double>  distances;
  std::unordered_map<int32_t, int32_t> parent;
  std::unordered_map<int32_t, int32_t> parent_link; // Track which link was used
  std::set<int32_t>                    visited;

  // Initialize
  distances[start_node] = 0.0;
  pq.emplace(0.0, start_node);

  size_t nodes_explored = 0;

  while (!pq.empty()) {
    if (config.max_nodes_explored &&
        nodes_explored >= *config.max_nodes_explored) {
      break;
    }

    auto [current_distance, current_node] = pq.top();
    pq.pop();

    if (visited.count(current_node)) {
      continue;
    }

    visited.insert(current_node);
    nodes_explored++;

    if (current_node == target_node) {
      // Found path
      auto node_path       = reconstructPath(parent, target_node);
      result.success       = true;
      result.totalDistance = current_distance;

      // Convert node IDs to NodeInfo
      for (int32_t node_id : node_path) {
        result.nodeSequence.push_back(getNode(node_id));
      }

      // Build link sequence
      for (size_t i = 0; i < node_path.size() - 1; ++i) {
        auto link_opt = getLinkBetweenNodes(node_path[i], node_path[i + 1]);
        if (link_opt) {
          result.linkSequence.push_back(getLink(*link_opt));
        }
      }

      // Convert to pose sequence
      for (int32_t node_id : node_path) {
        const auto &node = getNode(node_id);
        result.poseSequence.push_back(
            Eigen::Vector3d(node.x, node.y, node.theta));
      }

      return result;
    }

    // Explore neighbors
    for (int32_t link_id : getNodeNeighborLinks(current_node)) {
      const auto &link     = getLink(link_id);
      int32_t     neighbor = link.id_end;

      if (visited.count(neighbor)) {
        continue;
      }

      double edge_weight = getLinkLength(link_id);

      // Apply custom scorer if provided
      if (config.custom_scorer) {
        const auto &current_node_info  = getNode(current_node);
        const auto &neighbor_node_info = getNode(neighbor);
        edge_weight =
            config.custom_scorer(link, current_node_info, neighbor_node_info);
      }

      // Apply velocity constraints
      if (config.max_velocity && link.max_velocity > *config.max_velocity) {
        edge_weight *= 1.5; // Penalty for exceeding max velocity
      }

      double new_distance = current_distance + edge_weight;

      // Check search distance constraint
      if (config.max_search_distance &&
          new_distance > *config.max_search_distance) {
        continue;
      }

      if (distances.find(neighbor) == distances.end() ||
          new_distance < distances[neighbor]) {
        distances[neighbor]   = new_distance;
        parent[neighbor]      = current_node;
        parent_link[neighbor] = link_id;
        pq.emplace(new_distance, neighbor);
      }
    }
  }

  // No path found
  result.success = false;
  return result;
}

RichPathResult RichGraph::planAStar(int32_t start_node, int32_t target_node,
                                    const PlanningConfig &config) const {
  RichPathResult result;
  result.algorithmUsed = "A*";

  if (start_node == target_node) {
    result.nodeSequence  = {getNode(start_node)};
    result.success       = true;
    result.totalDistance = 0.0;

    const auto &node    = getNode(start_node);
    result.poseSequence = {Eigen::Vector3d(node.x, node.y, node.theta)};
    return result;
  }

  const auto &target_node_info = getNode(target_node);

  // Priority queue: (f_score, node_id)
  std::priority_queue<std::pair<double, int32_t>,
                      std::vector<std::pair<double, int32_t>>,
                      std::greater<std::pair<double, int32_t>>>
      pq;

  std::unordered_map<int32_t, double>  g_score; // Actual distance
  std::unordered_map<int32_t, double>  f_score; // g + heuristic
  std::unordered_map<int32_t, int32_t> parent;
  std::set<int32_t>                    closed_set;

  // Initialize
  g_score[start_node] = 0.0;
  f_score[start_node] = calculateHeuristic(
      getNode(start_node), target_node_info, PlanningConfig::Algorithm::A_STAR);
  pq.emplace(f_score[start_node], start_node);

  size_t nodes_explored = 0;

  while (!pq.empty()) {
    if (config.max_nodes_explored &&
        nodes_explored >= *config.max_nodes_explored) {
      break;
    }

    auto [current_f, current_node] = pq.top();
    pq.pop();

    if (closed_set.count(current_node)) {
      continue;
    }

    closed_set.insert(current_node);
    nodes_explored++;

    if (current_node == target_node) {
      // Found path
      auto node_path       = reconstructPath(parent, target_node);
      result.success       = true;
      result.totalDistance = g_score[target_node];

      // Convert node IDs to NodeInfo
      for (int32_t node_id : node_path) {
        result.nodeSequence.push_back(getNode(node_id));
      }

      // Build link sequence
      for (size_t i = 0; i < node_path.size() - 1; ++i) {
        auto link_opt = getLinkBetweenNodes(node_path[i], node_path[i + 1]);
        if (link_opt) {
          result.linkSequence.push_back(getLink(*link_opt));
        }
      }

      // Convert to pose sequence
      for (int32_t node_id : node_path) {
        const auto &node = getNode(node_id);
        result.poseSequence.push_back(
            Eigen::Vector3d(node.x, node.y, node.theta));
      }

      return result;
    }

    // Explore neighbors
    for (int32_t link_id : getNodeNeighborLinks(current_node)) {
      const auto &link     = getLink(link_id);
      int32_t     neighbor = link.id_end;

      if (closed_set.count(neighbor)) {
        continue;
      }

      double edge_weight = getLinkLength(link_id);

      // Apply custom scorer if provided
      if (config.custom_scorer) {
        const auto &current_node_info  = getNode(current_node);
        const auto &neighbor_node_info = getNode(neighbor);
        edge_weight =
            config.custom_scorer(link, current_node_info, neighbor_node_info);
      }

      double tentative_g = g_score[current_node] + edge_weight;

      if (g_score.find(neighbor) == g_score.end() ||
          tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        parent[neighbor]  = current_node;

        double h_score = calculateHeuristic(getNode(neighbor), target_node_info,
                                            PlanningConfig::Algorithm::A_STAR);
        f_score[neighbor] = tentative_g + h_score;

        pq.emplace(f_score[neighbor], neighbor);
      }
    }
  }

  // No path found
  result.success = false;
  return result;
}

RichPathResult RichGraph::planDirectPath(const Eigen::Vector3d &start_pose,
                                         int32_t                target_node,
                                         const PlanningConfig  &config) const {
  RichPathResult result;
  result.algorithmUsed = "DirectPath";

  const auto     &target_node_info = getNode(target_node);
  Eigen::Vector3d target_pose(target_node_info.x, target_node_info.y,
                              target_node_info.theta);

  // Simple direct path - just interpolate between start and target
  result.poseSequence  = {start_pose, target_pose};
  result.nodeSequence  = {getNode(target_node)}; // Only target node
  result.totalDistance = (target_pose.head<2>() - start_pose.head<2>()).norm();
  result.success       = true;

  return result;
}

RichPathResult RichGraph::planLinkBased(const Eigen::Vector3d &start_pose,
                                        int32_t                target_node,
                                        const PlanningConfig  &config) const {
  RichPathResult result;
  result.algorithmUsed = "LinkBased";

  // First, find closest links to start pose
  auto closest_links = findClosestLinks(start_pose, 5);

  if (closest_links.empty()) {
    result.success = false;
    return result;
  }

  // For each closest link, try to plan from its end node to target
  double         best_distance = std::numeric_limits<double>::max();
  RichPathResult best_path;

  for (int32_t link_id : closest_links) {
    const auto &link = getLink(link_id);

    // Project start pose onto this link
    double distance_to_link = getDistanceToLink(start_pose, link_id);

    // Plan from end of this link to target
    auto path_from_link = planDijkstra(link.id_end, target_node, config);

    if (path_from_link.success) {
      double total_distance = distance_to_link + path_from_link.totalDistance;

      if (total_distance < best_distance) {
        best_distance = total_distance;
        best_path     = path_from_link;

        // Prepend the link information
        best_path.linkSequence.insert(best_path.linkSequence.begin(), link);
        best_path.poseSequence.insert(best_path.poseSequence.begin(),
                                      start_pose);
        best_path.totalDistance = total_distance;
      }
    }
  }

  if (best_path.success) {
    result               = best_path;
    result.algorithmUsed = "LinkBased";
  } else {
    result.success = false;
  }

  return result;
}

// Path validation
bool RichGraph::isPathValid(const std::vector<int32_t> &node_sequence) const {
  if (node_sequence.empty()) {
    return false;
  }

  for (size_t i = 0; i < node_sequence.size() - 1; ++i) {
    if (!areNodesConnected(node_sequence[i], node_sequence[i + 1])) {
      return false;
    }
  }

  return true;
}

bool RichGraph::isPathValid(const RichPathResult &path_result) const {
  if (!path_result.success || path_result.nodeSequence.empty()) {
    return false;
  }

  // Convert NodeInfo sequence to node ID sequence for validation
  std::vector<int32_t> node_ids;
  for (const auto &node : path_result.nodeSequence) {
    node_ids.push_back(node.id);
  }

  return isPathValid(node_ids);
}

double RichGraph::calculatePathLength(
    const std::vector<int32_t> &node_sequence) const {
  if (node_sequence.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (size_t i = 0; i < node_sequence.size() - 1; ++i) {
    total_length += getNodeDistance(node_sequence[i], node_sequence[i + 1]);
  }

  return total_length;
}

// Statistics and validation methods
RichGraph::GraphStatistics RichGraph::getStatistics() const {
  GraphStatistics stats;
  stats.node_count = nodes_.size();
  stats.link_count = links_.size();

  // Calculate bounding box
  if (!nodes_.empty()) {
    auto first_node        = nodes_.begin()->second;
    stats.bounding_box_min = {first_node.x, first_node.y};
    stats.bounding_box_max = {first_node.x, first_node.y};

    for (const auto &[node_id, node] : nodes_) {
      stats.bounding_box_min.first =
          std::min(stats.bounding_box_min.first, static_cast<double>(node.x));
      stats.bounding_box_min.second =
          std::min(stats.bounding_box_min.second, static_cast<double>(node.y));
      stats.bounding_box_max.first =
          std::max(stats.bounding_box_max.first, static_cast<double>(node.x));
      stats.bounding_box_max.second =
          std::max(stats.bounding_box_max.second, static_cast<double>(node.y));
    }
  }

  // Calculate average node degree
  if (!nodes_.empty()) {
    size_t total_degree = 0;
    for (const auto &[node_id, node] : nodes_) {
      total_degree += getNodeNeighborLinks(node_id).size();
    }
    stats.average_node_degree =
        static_cast<double>(total_degree) / nodes_.size();
  }

  // Graph density = 2 * |E| / (|V| * (|V| - 1))
  if (nodes_.size() > 1) {
    stats.graph_density =
        (2.0 * links_.size()) / (nodes_.size() * (nodes_.size() - 1));
  }

  // Connected components (simplified - just check if graph is connected)
  stats.connected_components =
      1; // TODO: Implement proper connected components calculation

  return stats;
}

RichGraph::ValidationResult RichGraph::validateGraph() const {
  ValidationResult result;
  result.is_valid = true;

  // Check for orphaned nodes
  for (const auto &[node_id, node] : nodes_) {
    if (getNodeNeighborLinks(node_id).empty() &&
        getNodeIncomingLinks(node_id).empty()) {
      result.warnings.push_back("Orphaned node found: " +
                                std::to_string(node_id));
    }
  }

  // Check for invalid links
  for (const auto &[link_id, link] : links_) {
    if (!hasNode(link.id_start)) {
      result.errors.push_back(
          "Link " + std::to_string(link_id) +
          " has invalid start node: " + std::to_string(link.id_start));
      result.is_valid = false;
    }
    if (!hasNode(link.id_end)) {
      result.errors.push_back(
          "Link " + std::to_string(link_id) +
          " has invalid end node: " + std::to_string(link.id_end));
      result.is_valid = false;
    }
  }

  return result;
}

// Export/Import functionality
bool RichGraph::exportToJson(const std::string &filepath) const {
  try {
    nlohmann::json j;

    // Export nodes
    j["nodes"] = nlohmann::json::array();
    for (const auto &[node_id, node] : nodes_) {
      nlohmann::json node_json;
      node_json["id"]        = node.id;
      node_json["node_name"] = node.node_name;
      node_json["x"]         = node.x;
      node_json["y"]         = node.y;
      node_json["theta"]     = node.theta;
      node_json["type"]      = node.type;
      node_json["map_id"]    = node.map_id;
      j["nodes"].push_back(node_json);
    }

    // Export links
    j["links"] = nlohmann::json::array();
    for (const auto &[link_id, link] : links_) {
      nlohmann::json link_json;
      link_json["id_straight_link"] = link.id_straight_link;
      link_json["id_start"]         = link.id_start;
      link_json["id_end"]           = link.id_end;
      link_json["map_id"]           = link.map_id;
      link_json["max_velocity"]     = link.max_velocity;
      j["links"].push_back(link_json);
    }

    // Export metadata
    j["metadata"]["map_name"] = current_map_name_;
    j["metadata"]["map_id"]   = current_map_id_;

    std::ofstream file(filepath);
    file << j.dump(2);
    return true;

  } catch (const std::exception &e) {
    return false;
  }
}

bool RichGraph::importFromJson(const std::string &filepath) {
  try {
    std::ifstream  file(filepath);
    nlohmann::json j;
    file >> j;

    clear();

    // Import metadata
    if (j.contains("metadata")) {
      current_map_name_ = j["metadata"]["map_name"];
      current_map_id_   = j["metadata"]["map_id"];
    }

    // Import nodes
    if (j.contains("nodes")) {
      for (const auto &node_json : j["nodes"]) {
        NodeInfo node;
        node.id        = node_json["id"];
        node.node_name = node_json["node_name"];
        node.x         = node_json["x"];
        node.y         = node_json["y"];
        node.theta     = node_json["theta"];
        node.type      = node_json["type"];
        node.map_id    = node_json["map_id"];

        nodes_[node.id] = std::move(node);
      }
    }

    // Import links
    if (j.contains("links")) {
      for (const auto &link_json : j["links"]) {
        LinkInfo link;
        link.id_straight_link = link_json["id_straight_link"];
        link.id_start         = link_json["id_start"];
        link.id_end           = link_json["id_end"];
        link.map_id           = link_json["map_id"];
        link.max_velocity     = link_json["max_velocity"];

        links_[link.id_straight_link] = std::move(link);
      }
    }

    buildAdjacencyList();
    return true;

  } catch (const std::exception &e) {
    clear();
    return false;
  }
}

} // namespace vrobot_route_follow