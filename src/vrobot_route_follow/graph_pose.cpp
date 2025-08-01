#include "vrobot_route_follow/graph_pose.hpp"
#include <iostream>

namespace vrobot_route_follow {

// Explicit template instantiations for common types
template class GraphPose<int, CPose2D, double, double>;
template class GraphPose<long, CPose2D, double, double>;
template class GraphPose<unsigned long, CPose2D, double, double>;

// Implementation of template methods that were previously in header
template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
auto GraphPose<NodeID, Pose2D, WeightType, VelocityType>::planPath(
    const Pose2D &startPose, const NodeID &targetNode,
    const PlanningConfig &config) const -> PlanningResult {

  PlanningResult result;

  if (!Base::hasNode(targetNode)) {
    result.algorithmUsed = "INVALID_TARGET";
    result.errorMessage  = "Target node " + std::to_string(targetNode) +
                          " does not exist in graph";
    return result;
  }

  const Pose2D &targetPose     = Base::getNodePose(targetNode);
  double        directDistance = startPose.distanceTo(targetPose);

  // Check for direct path first
  if (directDistance <= config.directThreshold) {
    result.pathSegments = {
        {startPose, targetPose}
    };

    result.vpathSegments = {
        {{startPose, targetPose}, 0.3}
    };

    result.totalDistance               = directDistance;
    result.algorithmUsed               = "DIRECT";
    result.success                     = true;
    result.metadata["direct_distance"] = directDistance;
    return result;
  }

  // Check if close enough for standard Dijkstra
  double distanceToClosest = this->getDistanceToClosestNode(startPose);
  if (!config.enableLinkBased ||
      distanceToClosest <= config.distanceThreshold) {
    auto [path, distance] = this->dijkstraFromPose(startPose, targetNode);
    if (distance) {
      result.pathSegments                    = path;
      result.vpathSegments                   = this->pathSegmentsToVPath(path);
      result.totalDistance                   = distance;
      result.algorithmUsed                   = "DIJKSTRA_FROM_POSE";
      result.success                         = true;
      result.metadata["distance_to_closest"] = distanceToClosest;
      return result;
    }
  }

  // Use advanced link-based approach
  auto [linkPath, linkDistance] = this->dijkstraWithModularLinkApproach(
      startPose, targetNode, config.directThreshold, config.maxLinks,
      config.linkDistanceWeight, config.maxLinkDistance,
      config.graphDistanceWeight);

  if (linkDistance) {
    result.pathSegments                  = linkPath;
    result.vpathSegments                 = this->pathSegmentsToVPath(linkPath);
    result.totalDistance                 = linkDistance;
    result.algorithmUsed                 = "MODULAR_LINK_BASED";
    result.success                       = true;
    result.metadata["link_weight"]       = config.linkDistanceWeight;
    result.metadata["max_link_distance"] = config.maxLinkDistance;
    return result;
  }

  // Fallback to basic approach
  auto [fallbackPath, fallbackDistance] =
      this->dijkstraFromPose(startPose, targetNode, config.maxLinkDistance);
  if (fallbackDistance) {
    result.pathSegments  = fallbackPath;
    result.vpathSegments = this->pathSegmentsToVPath(fallbackPath);
    result.totalDistance = fallbackDistance;
    result.algorithmUsed = "DIJKSTRA_FALLBACK";
    result.success       = true;
    return result;
  }

  result.algorithmUsed = "FAILED";
  result.errorMessage =
      "All pathfinding algorithms failed. Distance to closest node: " +
      std::to_string(distanceToClosest) +
      ", Max distance threshold: " + std::to_string(config.maxLinkDistance) +
      ", Direct distance: " + std::to_string(directDistance) +
      ", Direct threshold: " + std::to_string(config.directThreshold);
  return result;
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
nav_msgs::msg::Path
GraphPose<NodeID, Pose2D, WeightType, VelocityType>::planningResultToNavPath(
    const PlanningResult &result, const std::string &frameId,
    const rclcpp::Time &timestamp, double resolution) const {

  if (!result.success) {
    nav_msgs::msg::Path emptyPath;
    emptyPath.header.frame_id = frameId;
    emptyPath.header.stamp    = timestamp;
    return emptyPath;
  }

  return this->toNavPath(result.pathSegments, frameId, timestamp, resolution);
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
vrobot_local_planner::msg::Path
GraphPose<NodeID, Pose2D, WeightType, VelocityType>::planningResultToVPath(
    const PlanningResult &result, const std::string &frameId,
    const rclcpp::Time &timestamp, double resolution) const {

  if (!result.success) {
    vrobot_local_planner::msg::Path emptyPath;
    emptyPath.header.frame_id = frameId;
    emptyPath.header.stamp    = timestamp;
    return emptyPath;
  }

  return this->toVPath(result.vpathSegments, frameId, timestamp, resolution);
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
std::vector<
    typename GraphPose<NodeID, Pose2D, WeightType, VelocityType>::VPathSegment>
GraphPose<NodeID, Pose2D, WeightType, VelocityType>::pathSegmentsToVPath(
    const std::vector<PathSegment> &pathSegments) const {

  std::vector<VPathSegment> vpathSegments;
  vpathSegments.reserve(pathSegments.size());

  for (const auto &segment : pathSegments) {
    // Find closest nodes to segment start and end to determine velocity
    NodeID startNode = this->getClosestNode(segment.first);
    NodeID endNode   = this->getClosestNode(segment.second);

    // Get velocity for this edge if it exists
    double velocity = 1.0; // Default velocity
    if (this->hasEdgeVelocity(startNode, endNode)) {
      velocity = this->getEdgeVelocity(startNode, endNode);
    }

    vpathSegments.emplace_back(segment, velocity);
  }

  return vpathSegments;
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
void GraphPose<NodeID, Pose2D, WeightType, VelocityType>::debugClosestLinks(
    const Pose2D &queryPose, size_t maxLinks, double maxDistance) const {

  std::cout << "\n=== DEBUG: Closest Links Analysis ===" << std::endl;
  std::cout << "Query pose: (" << queryPose.x() << ", " << queryPose.y() << ")"
            << std::endl;

  auto closestLinks = this->getClosestLinks(queryPose, maxLinks, maxDistance);

  if (closestLinks.empty()) {
    std::cout << "No links found within distance threshold" << std::endl;
    return;
  }

  std::cout << "Found " << closestLinks.size()
            << " closest links:" << std::endl;

  for (size_t i = 0; i < closestLinks.size(); ++i) {
    const auto &[linkStart, linkEnd, distance] = closestLinks[i];

    const Pose2D &startPose = Base::getNodePose(linkStart);
    const Pose2D &endPose   = Base::getNodePose(linkEnd);

    std::cout << "  " << (i + 1) << ". Link " << linkStart << "→" << linkEnd
              << " (distance: " << distance << ")" << std::endl;
    std::cout << "     Start: (" << startPose.x() << ", " << startPose.y()
              << ")" << std::endl;
    std::cout << "     End: (" << endPose.x() << ", " << endPose.y() << ")"
              << std::endl;

    Pose2D projection = this->projectOntoLink(queryPose, linkStart, linkEnd);
    std::cout << "     Projection: (" << projection.x() << ", "
              << projection.y() << ")" << std::endl;
  }
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
std::map<std::string, double>
GraphPose<NodeID, Pose2D, WeightType, VelocityType>::getGraphStatistics()
    const {
  std::map<std::string, double> stats;

  stats["node_count"] = static_cast<double>(Base::getNodeCount());
  stats["edge_count"] = static_cast<double>(Base::getEdgeCount());

  if (Base::empty()) {
    return stats;
  }

  // Calculate graph bounds
  double minX = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double minY = std::numeric_limits<double>::max();
  double maxY = std::numeric_limits<double>::lowest();

  for (const auto &[nodeId, pose] : Base::nodePoses_) {
    minX = std::min(minX, pose.x());
    maxX = std::max(maxX, pose.x());
    minY = std::min(minY, pose.y());
    maxY = std::max(maxY, pose.y());
  }

  stats["min_x"]  = minX;
  stats["max_x"]  = maxX;
  stats["min_y"]  = minY;
  stats["max_y"]  = maxY;
  stats["width"]  = maxX - minX;
  stats["height"] = maxY - minY;

  // Calculate average node degree
  double totalDegree = 0.0;
  for (const auto &[nodeId, neighbors] : Base::adjList_) {
    totalDegree += neighbors.size();
  }
  stats["average_out_degree"] = totalDegree / Base::getNodeCount();

  return stats;
}

template <typename NodeID, typename Pose2D, typename WeightType,
          typename VelocityType>
std::pair<std::vector<typename GraphPose<NodeID, Pose2D, WeightType,
                                         VelocityType>::PathSegment>,
          std::optional<double>>
GraphPose<NodeID, Pose2D, WeightType, VelocityType>::planPathToNode(
    const Pose2D &startPose, const NodeID &targetNode) const {

  auto result = planPath(startPose, targetNode);
  if (result.success) {
    return {result.pathSegments, result.totalDistance};
  } else {
    return {std::vector<PathSegment>{}, std::nullopt};
  }
}

// Explicit instantiations for the template methods
template auto
GraphPose<int, CPose2D, double, double>::planPath(const CPose2D &, const int &,
                                                  const PlanningConfig &) const
    -> PlanningResult;

template nav_msgs::msg::Path
GraphPose<int, CPose2D, double, double>::planningResultToNavPath(
    const PlanningResult &, const std::string &, const rclcpp::Time &,
    double) const;

template vrobot_local_planner::msg::Path
GraphPose<int, CPose2D, double, double>::planningResultToVPath(
    const PlanningResult &, const std::string &, const rclcpp::Time &,
    double) const;

template void GraphPose<int, CPose2D, double, double>::debugClosestLinks(
    const CPose2D &, size_t, double) const;

template std::map<std::string, double>
GraphPose<int, CPose2D, double, double>::getGraphStatistics() const;

template std::pair<
    std::vector<GraphPose<int, CPose2D, double, double>::PathSegment>,
    std::optional<double>>
GraphPose<int, CPose2D, double, double>::planPathToNode(const CPose2D &,
                                                        const int &) const;

// Explicit instantiations for unsigned long (TNodeID type)
template auto GraphPose<unsigned long, CPose2D, double, double>::planPath(
    const CPose2D &, const unsigned long &, const PlanningConfig &) const
    -> PlanningResult;

template nav_msgs::msg::Path
GraphPose<unsigned long, CPose2D, double, double>::planningResultToNavPath(
    const PlanningResult &, const std::string &, const rclcpp::Time &,
    double) const;

template vrobot_local_planner::msg::Path
GraphPose<unsigned long, CPose2D, double, double>::planningResultToVPath(
    const PlanningResult &, const std::string &, const rclcpp::Time &,
    double) const;

template void
GraphPose<unsigned long, CPose2D, double, double>::debugClosestLinks(
    const CPose2D &, size_t, double) const;

template std::map<std::string, double>
GraphPose<unsigned long, CPose2D, double, double>::getGraphStatistics() const;

template std::pair<
    std::vector<GraphPose<unsigned long, CPose2D, double, double>::PathSegment>,
    std::optional<double>>
GraphPose<unsigned long, CPose2D, double, double>::planPathToNode(
    const CPose2D &, const unsigned long &) const;

// Explicit instantiations for pathSegmentsToVPath
template std::vector<GraphPose<int, CPose2D, double, double>::VPathSegment>
GraphPose<int, CPose2D, double, double>::pathSegmentsToVPath(
    const std::vector<PathSegment> &) const;

template std::vector<
    GraphPose<unsigned long, CPose2D, double, double>::VPathSegment>
GraphPose<unsigned long, CPose2D, double, double>::pathSegmentsToVPath(
    const std::vector<PathSegment> &) const;

} // namespace vrobot_route_follow