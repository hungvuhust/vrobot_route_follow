#include "vrobot_route_follow/algorithms/rich_link_based_planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace vrobot_route_follow {
namespace algorithms {

RichLinkBasedPlanner::RichLinkBasedPlanner(
    std::shared_ptr<GraphDataInterface> graph_data,
    std::shared_ptr<RichPathfinding>    pathfinding)
    : graph_data_(std::move(graph_data)), pathfinding_(std::move(pathfinding)) {
  if (!graph_data_) {
    throw std::invalid_argument("Graph data interface cannot be null");
  }
}

std::vector<LinkConnection> RichLinkBasedPlanner::findNearbyLinks(
    const Eigen::Vector3d &pose, const LinkBasedPlanningConfig &config) const {
  std::vector<LinkConnection> connections;

  // Find nodes within connection distance
  auto nearby_nodes =
      graph_data_->findNodesInRadius(pose, config.max_connection_distance);

  // Collect all links connected to nearby nodes
  std::unordered_set<int32_t> processed_links;
  for (int32_t node_id : nearby_nodes) {
    auto node_links = graph_data_->getNodeNeighborLinks(node_id);
    for (int32_t link_id : node_links) {
      if (processed_links.count(link_id))
        continue;
      processed_links.insert(link_id);

      if (!graph_data_->hasLink(link_id))
        continue;

      const LinkInfo &link = graph_data_->getLink(link_id);

      // Project pose onto link
      Eigen::Vector3d projection = projectPointOntoLink(pose, link);
      double          distance = (projection.head<2>() - pose.head<2>()).norm();

      if (distance <= config.max_connection_distance) {
        // Calculate projection parameter
        const NodeInfo &start_node = graph_data_->getNode(link.id_start);
        const NodeInfo &end_node   = graph_data_->getNode(link.id_end);

        Eigen::Vector3d start_pos(start_node.x, start_node.y, start_node.theta);
        Eigen::Vector3d end_pos(end_node.x, end_node.y, end_node.theta);

        double link_length = (end_pos.head<2>() - start_pos.head<2>()).norm();
        double projection_length =
            (projection.head<2>() - start_pos.head<2>()).norm();
        double param =
            (link_length > 0) ? projection_length / link_length : 0.0;
        param = std::clamp(param, 0.0, 1.0);

        connections.emplace_back(link_id, projection, distance, param);
      }
    }
  }

  // Sort by connection distance
  std::sort(connections.begin(), connections.end(),
            [](const LinkConnection &a, const LinkConnection &b) {
              return a.connection_distance < b.connection_distance;
            });

  return connections;
}

LinkConnection RichLinkBasedPlanner::findBestLinkConnection(
    const Eigen::Vector3d &pose, const std::vector<LinkConnection> &candidates,
    const LinkBasedPlanningConfig &config) const {
  if (candidates.empty()) {
    throw std::runtime_error("No link connection candidates provided");
  }

  LinkConnection best_connection = candidates[0];
  double         best_score      = std::numeric_limits<double>::max();

  for (const auto &connection : candidates) {
    const LinkInfo &link = graph_data_->getLink(connection.link_id);

    // Calculate score using custom scorer if provided
    double score;
    if (config.custom_link_scorer) {
      score =
          config.custom_link_scorer(link, pose, connection.connection_point);
    } else {
      // Default scoring: distance + link length penalty
      score = connection.connection_distance;

      if (config.prefer_shorter_links) {
        const NodeInfo &start_node = graph_data_->getNode(link.id_start);
        const NodeInfo &end_node   = graph_data_->getNode(link.id_end);
        Eigen::Vector3d start_pos(start_node.x, start_node.y, start_node.theta);
        Eigen::Vector3d end_pos(end_node.x, end_node.y, end_node.theta);
        double link_length = (end_pos.head<2>() - start_pos.head<2>()).norm();
        score += 0.1 * link_length; // Small penalty for longer links
      }
    }

    if (score < best_score) {
      best_score      = score;
      best_connection = connection;
    }
  }

  return best_connection;
}

Eigen::Vector3d
RichLinkBasedPlanner::projectPointOntoLink(const Eigen::Vector3d &point,
                                           const LinkInfo        &link) const {
  const NodeInfo &start_node = graph_data_->getNode(link.id_start);
  const NodeInfo &end_node   = graph_data_->getNode(link.id_end);

  Eigen::Vector3d start_pos(start_node.x, start_node.y, start_node.theta);
  Eigen::Vector3d end_pos(end_node.x, end_node.y, end_node.theta);

  // Project point onto line segment
  Eigen::Vector2d line_vec  = end_pos.head<2>() - start_pos.head<2>();
  Eigen::Vector2d point_vec = point.head<2>() - start_pos.head<2>();

  double line_length_sq = line_vec.squaredNorm();
  if (line_length_sq < 1e-6) {
    // Degenerate case: start and end are the same
    return start_pos;
  }

  double t = point_vec.dot(line_vec) / line_length_sq;
  t        = std::clamp(t, 0.0, 1.0);

  Eigen::Vector2d projection = start_pos.head<2>() + t * line_vec;

  // Interpolate theta
  double theta = start_pos(2) + t * (end_pos(2) - start_pos(2));

  return Eigen::Vector3d(projection(0), projection(1), theta);
}

double RichLinkBasedPlanner::calculateLinkScore(
    const LinkInfo &link, const Eigen::Vector3d &start_pose,
    const Eigen::Vector3d         &target_pose,
    const LinkBasedPlanningConfig &config) const {
  // Basic score: link length
  const NodeInfo &start_node = graph_data_->getNode(link.id_start);
  const NodeInfo &end_node   = graph_data_->getNode(link.id_end);

  Eigen::Vector3d link_start(start_node.x, start_node.y, start_node.theta);
  Eigen::Vector3d link_end(end_node.x, end_node.y, end_node.theta);

  double link_length = (link_end.head<2>() - link_start.head<2>()).norm();
  double score       = link_length;

  // Add penalties based on configuration
  if (config.avoid_congested_links) {
    // Placeholder: could be based on dynamic information
    score *= config.congestion_penalty;
  }

  return score;
}

std::vector<LinkPathSegment> RichLinkBasedPlanner::buildLinkPath(
    const LinkConnection          &start_connection,
    const LinkConnection          &target_connection,
    const LinkBasedPlanningConfig &config) const {
  std::vector<LinkPathSegment> segments;

  if (start_connection.link_id == target_connection.link_id) {
    // Same link: create single segment
    LinkPathSegment segment(start_connection.connection_point,
                            target_connection.connection_point, "link_follow");
    segment.link_id = start_connection.link_id;
    segments.push_back(segment);
  } else {
    // Different links: need to plan path between them
    const LinkInfo &start_link = graph_data_->getLink(start_connection.link_id);
    const LinkInfo &target_link =
        graph_data_->getLink(target_connection.link_id);

    // Find path between links using node-based planning
    int32_t start_node  = (start_connection.projection_parameter < 0.5)
                              ? start_link.id_start
                              : start_link.id_end;
    int32_t target_node = (target_connection.projection_parameter < 0.5)
                              ? target_link.id_start
                              : target_link.id_end;

    // Create segments: start -> start_node -> ... -> target_node -> target
    LinkPathSegment start_segment(
        start_connection.connection_point,
        Eigen::Vector3d(graph_data_->getNode(start_node).x,
                        graph_data_->getNode(start_node).y,
                        graph_data_->getNode(start_node).theta),
        "link_follow");
    start_segment.link_id     = start_connection.link_id;
    start_segment.end_node_id = start_node;
    segments.push_back(start_segment);

    // Use pathfinding for middle section if available
    if (pathfinding_ && start_node != target_node) {
      PathfindingConfig pf_config;
      pf_config.max_connection_distance = config.max_connection_distance;
      auto path_result =
          pathfinding_->planDijkstra(start_node, target_node, pf_config);

      if (path_result.success && path_result.nodeSequence.size() > 1) {
        for (size_t i = 1; i < path_result.nodeSequence.size(); ++i) {
          const NodeInfo &from_node = path_result.nodeSequence[i - 1];
          const NodeInfo &to_node   = path_result.nodeSequence[i];

          LinkPathSegment segment(
              Eigen::Vector3d(from_node.x, from_node.y, from_node.theta),
              Eigen::Vector3d(to_node.x, to_node.y, to_node.theta),
              "node_connection");
          segment.start_node_id = from_node.id;
          segment.end_node_id   = to_node.id;

          if (i - 1 < path_result.linkSequence.size()) {
            segment.link_id = path_result.linkSequence[i - 1].id_straight_link;
          }

          segments.push_back(segment);
        }
      } else {
        // Fallback: direct connection
        LinkPathSegment middle_segment(
            Eigen::Vector3d(graph_data_->getNode(start_node).x,
                            graph_data_->getNode(start_node).y,
                            graph_data_->getNode(start_node).theta),
            Eigen::Vector3d(graph_data_->getNode(target_node).x,
                            graph_data_->getNode(target_node).y,
                            graph_data_->getNode(target_node).theta),
            "direct");
        middle_segment.start_node_id = start_node;
        middle_segment.end_node_id   = target_node;
        segments.push_back(middle_segment);
      }
    }

    // Final segment: target_node -> target
    LinkPathSegment target_segment(
        Eigen::Vector3d(graph_data_->getNode(target_node).x,
                        graph_data_->getNode(target_node).y,
                        graph_data_->getNode(target_node).theta),
        target_connection.connection_point, "link_follow");
    target_segment.link_id       = target_connection.link_id;
    target_segment.start_node_id = target_node;
    segments.push_back(target_segment);
  }

  return segments;
}

std::vector<LinkPathSegment> RichLinkBasedPlanner::buildDirectPath(
    const Eigen::Vector3d &start_pose,
    const Eigen::Vector3d &target_pose) const {
  std::vector<LinkPathSegment> segments;
  segments.emplace_back(start_pose, target_pose, "direct");
  return segments;
}

RichPathResult RichLinkBasedPlanner::convertToRichPathResult(
    const std::vector<LinkPathSegment> &segments,
    const std::string                  &algorithm_name) const {
  RichPathResult result;
  result.success       = true;
  result.algorithmUsed = algorithm_name;
  result.totalDistance = 0.0;

  // Convert segments to poses and calculate total distance
  for (const auto &segment : segments) {
    result.poseSequence.push_back(segment.start_pose);
    result.totalDistance += segment.segment_length;

    // Add node information if available
    if (segment.start_node_id.has_value() &&
        graph_data_->hasNode(segment.start_node_id.value())) {
      result.nodeSequence.push_back(
          graph_data_->getNode(segment.start_node_id.value()));
    }

    // Add link information if available
    if (segment.link_id.has_value() &&
        graph_data_->hasLink(segment.link_id.value())) {
      result.linkSequence.push_back(
          graph_data_->getLink(segment.link_id.value()));
    }
  }

  // Add final pose
  if (!segments.empty()) {
    result.poseSequence.push_back(segments.back().end_pose);

    // Add final node if available
    if (segments.back().end_node_id.has_value() &&
        graph_data_->hasNode(segments.back().end_node_id.value())) {
      result.nodeSequence.push_back(
          graph_data_->getNode(segments.back().end_node_id.value()));
    }
  }

  return result;
}

bool RichLinkBasedPlanner::validateLinkPath(
    const std::vector<LinkPathSegment> &segments) const {
  if (segments.empty())
    return false;

  // Check continuity
  for (size_t i = 1; i < segments.size(); ++i) {
    double gap =
        (segments[i].start_pose.head<2>() - segments[i - 1].end_pose.head<2>())
            .norm();
    if (gap > 0.1) { // 10cm tolerance
      return false;
    }
  }

  return true;
}

std::vector<LinkPathSegment> RichLinkBasedPlanner::optimizePath(
    const std::vector<LinkPathSegment> &segments,
    const LinkBasedPlanningConfig      &config) const {
  if (!config.optimize_path_smoothness) {
    return segments;
  }

  // Simple optimization: remove unnecessary waypoints
  std::vector<LinkPathSegment> optimized;
  if (segments.empty())
    return optimized;

  optimized.push_back(segments[0]);

  for (size_t i = 1; i < segments.size(); ++i) {
    const auto &prev = optimized.back();
    const auto &curr = segments[i];

    // Check if we can skip intermediate waypoints
    double angle_change =
        std::abs(std::atan2(curr.end_pose(1) - prev.start_pose(1),
                            curr.end_pose(0) - prev.start_pose(0)) -
                 std::atan2(prev.end_pose(1) - prev.start_pose(1),
                            prev.end_pose(0) - prev.start_pose(0)));

    if (angle_change < 0.1) { // Small angle change, can merge
      optimized.back().end_pose       = curr.end_pose;
      optimized.back().segment_length = (optimized.back().end_pose.head<2>() -
                                         optimized.back().start_pose.head<2>())
                                            .norm();
    } else {
      optimized.push_back(curr);
    }
  }

  return optimized;
}

RichPathResult RichLinkBasedPlanner::tryNodeBasedFallback(
    const Eigen::Vector3d &start_pose, const Eigen::Vector3d &target_pose,
    const LinkBasedPlanningConfig &config) const {
  if (!pathfinding_) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "No pathfinding instance available for fallback";
    return result;
  }

  // Find closest nodes
  int32_t start_node  = graph_data_->findClosestNode(start_pose);
  int32_t target_node = graph_data_->findClosestNode(target_pose);

  // Try different algorithms based on configuration
  if (config.use_astar_fallback) {
    PathfindingConfig pf_config;
    pf_config.max_connection_distance = config.max_connection_distance;
    auto result = pathfinding_->planAStar(start_node, target_node, pf_config);
    if (result.success) {
      result.algorithmUsed = "LinkBased_AStar_Fallback";
      return result;
    }
  }

  if (config.use_dijkstra_fallback) {
    PathfindingConfig pf_config;
    pf_config.max_connection_distance = config.max_connection_distance;
    auto result =
        pathfinding_->planDijkstra(start_node, target_node, pf_config);
    if (result.success) {
      result.algorithmUsed = "LinkBased_Dijkstra_Fallback";
      return result;
    }
  }

  RichPathResult result;
  result.success      = false;
  result.errorMessage = "All fallback algorithms failed";
  return result;
}

RichPathResult
RichLinkBasedPlanner::planPath(const Eigen::Vector3d         &start_pose,
                               const Eigen::Vector3d         &target_pose,
                               const LinkBasedPlanningConfig &config) const {
  auto start_time = std::chrono::high_resolution_clock::now();
  last_stats_     = LinkPlanningStats{};

  RichPathResult result;
  result.algorithmUsed = "LinkBased";
  result.success       = false;

  // Check if direct connection is possible
  double direct_distance =
      (target_pose.head<2>() - start_pose.head<2>()).norm();
  if (direct_distance <= config.max_connection_distance &&
      config.prefer_direct_connections) {
    auto segments = buildDirectPath(start_pose, target_pose);
    result        = convertToRichPathResult(segments, "LinkBased_Direct");
    last_stats_.planning_strategy = "direct";

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);
    last_stats_.computation_time_ms = duration.count() / 1000.0;

    return result;
  }

  // Find nearby links for start and target
  auto start_connections  = findNearbyLinks(start_pose, config);
  auto target_connections = findNearbyLinks(target_pose, config);

  last_stats_.links_evaluated =
      start_connections.size() + target_connections.size();

  if (start_connections.empty() || target_connections.empty()) {
    // No links found, try fallback
    result = tryNodeBasedFallback(start_pose, target_pose, config);
    last_stats_.used_fallback      = true;
    last_stats_.fallback_algorithm = result.algorithmUsed;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);
    last_stats_.computation_time_ms = duration.count() / 1000.0;

    return result;
  }

  // Find best connections
  auto best_start_connection =
      findBestLinkConnection(start_pose, start_connections, config);
  auto best_target_connection =
      findBestLinkConnection(target_pose, target_connections, config);

  last_stats_.connections_tested = 2;

  // Build path using links
  auto segments =
      buildLinkPath(best_start_connection, best_target_connection, config);

  if (validateLinkPath(segments)) {
    segments = optimizePath(segments, config);
    result   = convertToRichPathResult(segments, "LinkBased");

    if (segments.size() == 1) {
      last_stats_.planning_strategy = "single_link";
    } else {
      last_stats_.planning_strategy = "multi_link";
    }
  } else {
    // Path validation failed, try fallback
    result = tryNodeBasedFallback(start_pose, target_pose, config);
    last_stats_.used_fallback      = true;
    last_stats_.fallback_algorithm = result.algorithmUsed;
    last_stats_.planning_strategy  = "node_based";
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  last_stats_.computation_time_ms = duration.count() / 1000.0;

  return result;
}

RichPathResult RichLinkBasedPlanner::planPathToNode(
    const Eigen::Vector3d &start_pose, int32_t target_node,
    const LinkBasedPlanningConfig &config) const {
  if (!graph_data_->hasNode(target_node)) {
    RichPathResult result;
    result.success = false;
    result.errorMessage =
        "Target node not found: " + std::to_string(target_node);
    return result;
  }

  const NodeInfo &target_node_info = graph_data_->getNode(target_node);
  Eigen::Vector3d target_pose(target_node_info.x, target_node_info.y,
                              target_node_info.theta);

  return planPath(start_pose, target_pose, config);
}

RichPathResult RichLinkBasedPlanner::planPathFromNode(
    int32_t start_node, const Eigen::Vector3d &target_pose,
    const LinkBasedPlanningConfig &config) const {
  if (!graph_data_->hasNode(start_node)) {
    RichPathResult result;
    result.success      = false;
    result.errorMessage = "Start node not found: " + std::to_string(start_node);
    return result;
  }

  const NodeInfo &start_node_info = graph_data_->getNode(start_node);
  Eigen::Vector3d start_pose(start_node_info.x, start_node_info.y,
                             start_node_info.theta);

  return planPath(start_pose, target_pose, config);
}

std::vector<LinkConnection> RichLinkBasedPlanner::analyzeLinkConnectivity(
    const Eigen::Vector3d &pose, const LinkBasedPlanningConfig &config) const {
  auto connections = findNearbyLinks(pose, config);

  // Sort by score for analysis
  std::sort(
      connections.begin(), connections.end(),
      [this, &pose, &config](const LinkConnection &a, const LinkConnection &b) {
        const LinkInfo &link_a = graph_data_->getLink(a.link_id);
        const LinkInfo &link_b = graph_data_->getLink(b.link_id);

        double score_a =
            calculateLinkScore(link_a, pose, a.connection_point, config);
        double score_b =
            calculateLinkScore(link_b, pose, b.connection_point, config);

        return score_a < score_b;
      });

  return connections;
}

// Factory methods
std::unique_ptr<RichLinkBasedPlanner>
LinkBasedPlannerFactory::create(std::shared_ptr<GraphDataInterface> graph_data,
                                std::shared_ptr<RichPathfinding> pathfinding) {
  return std::make_unique<RichLinkBasedPlanner>(graph_data, pathfinding);
}

LinkBasedPlanningConfig LinkBasedPlannerFactory::createDefaultConfig() {
  return LinkBasedPlanningConfig{};
}

LinkBasedPlanningConfig
LinkBasedPlannerFactory::createDirectConnectionConfig() {
  LinkBasedPlanningConfig config;
  config.prefer_direct_connections = true;
  config.max_connection_distance   = 10.0;
  config.use_dijkstra_fallback     = false;
  config.use_astar_fallback        = false;
  return config;
}

LinkBasedPlanningConfig LinkBasedPlannerFactory::createLinkFollowingConfig() {
  LinkBasedPlanningConfig config;
  config.prefer_direct_connections = false;
  config.prefer_shorter_links      = true;
  config.optimize_path_smoothness  = true;
  config.projection_tolerance      = 0.1;
  return config;
}

LinkBasedPlanningConfig LinkBasedPlannerFactory::createRobustConfig() {
  LinkBasedPlanningConfig config;
  config.use_dijkstra_fallback   = true;
  config.use_astar_fallback      = true;
  config.max_fallback_attempts   = 5;
  config.max_connection_distance = 8.0;
  return config;
}

// Utility functions
namespace link_planning_utils {

std::pair<Eigen::Vector3d, double> calculateOptimalConnectionPoint(
    const Eigen::Vector3d &pose, const LinkInfo &link,
    const NodeInfo &start_node, const NodeInfo &end_node) {

  Eigen::Vector3d start_pos(start_node.x, start_node.y, start_node.theta);
  Eigen::Vector3d end_pos(end_node.x, end_node.y, end_node.theta);

  // Project point onto line segment
  Eigen::Vector2d line_vec  = end_pos.head<2>() - start_pos.head<2>();
  Eigen::Vector2d point_vec = pose.head<2>() - start_pos.head<2>();

  double line_length_sq = line_vec.squaredNorm();
  if (line_length_sq < 1e-6) {
    return {start_pos, 0.0};
  }

  double t = point_vec.dot(line_vec) / line_length_sq;
  t        = std::clamp(t, 0.0, 1.0);

  Eigen::Vector2d projection = start_pos.head<2>() + t * line_vec;
  double          theta      = start_pos(2) + t * (end_pos(2) - start_pos(2));

  return {Eigen::Vector3d(projection(0), projection(1), theta), t};
}

bool canConnectDirectly(const Eigen::Vector3d &pose1,
                        const Eigen::Vector3d &pose2, double max_distance,
                        double max_angle_diff) {
  double distance = (pose2.head<2>() - pose1.head<2>()).norm();
  if (distance > max_distance)
    return false;

  double angle_diff = std::abs(pose2(2) - pose1(2));
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  if (angle_diff > M_PI)
    angle_diff = 2 * M_PI - angle_diff;

  return angle_diff <= max_angle_diff;
}

std::vector<LinkPathSegment>
smoothPath(const std::vector<LinkPathSegment> &segments,
           double                              smoothing_tolerance) {
  if (segments.size() <= 2)
    return segments;

  std::vector<LinkPathSegment> smoothed;
  smoothed.push_back(segments[0]);

  for (size_t i = 1; i < segments.size() - 1; ++i) {
    const auto &prev = segments[i - 1];
    const auto &curr = segments[i];
    const auto &next = segments[i + 1];

    // Calculate deviation from straight line
    Eigen::Vector2d direct_vec =
        next.end_pose.head<2>() - prev.start_pose.head<2>();
    Eigen::Vector2d via_point =
        curr.end_pose.head<2>() - prev.start_pose.head<2>();

    double deviation =
        std::abs(direct_vec.normalized().dot(via_point.normalized()) - 1.0);

    if (deviation > smoothing_tolerance) {
      smoothed.push_back(curr);
    }
  }

  smoothed.push_back(segments.back());
  return smoothed;
}

PathValidationResult validatePath(const std::vector<LinkPathSegment> &segments,
                                  double                              max_gap) {
  PathValidationResult result;

  if (segments.empty()) {
    result.is_valid = false;
    result.errors.push_back("Empty path");
    return result;
  }

  // Check continuity
  for (size_t i = 1; i < segments.size(); ++i) {
    double gap =
        (segments[i].start_pose.head<2>() - segments[i - 1].end_pose.head<2>())
            .norm();
    if (gap > max_gap) {
      result.errors.push_back("Gap too large between segments " +
                              std::to_string(i - 1) + " and " +
                              std::to_string(i) + ": " + std::to_string(gap));
    }
  }

  // Check for zero-length segments
  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].segment_length < 1e-6) {
      result.warnings.push_back("Zero-length segment at index " +
                                std::to_string(i));
    }
  }

  result.is_valid = result.errors.empty();
  return result;
}

} // namespace link_planning_utils

} // namespace algorithms
} // namespace vrobot_route_follow