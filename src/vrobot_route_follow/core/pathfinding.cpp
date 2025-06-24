#include "vrobot_route_follow/core/pathfinding.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

namespace mrpt_graphPose_pose {
namespace core {

// Explicit template instantiations for common types
template class Pathfinding<int, CPose2D, double>;
template class Pathfinding<long, CPose2D, double>;
template class Pathfinding<unsigned long, CPose2D, double>;

// Implementation for commonly used methods can be added here
// Currently, most methods are inline in the header

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
Pathfinding<NodeID, Pose2D, WeightType>::dijkstra(
    const NodeID &startNode, const NodeID &targetNode) const {

  if (!Base::hasNode(startNode) || !Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  if (startNode == targetNode) {
    return {{}, 0.0};
  }

  // Initialize distances and predecessors
  struct NodeInfo {
    double                distance    = std::numeric_limits<double>::infinity();
    std::optional<NodeID> predecessor = std::nullopt;
  };

  std::unordered_map<NodeID, NodeInfo> nodeInfo;
  for (const NodeID &nodeId : Base::getAllNodeIds()) {
    nodeInfo[nodeId] = NodeInfo{};
  }
  nodeInfo[startNode].distance = 0.0;

  // Priority queue for Dijkstra
  using PQItem = std::pair<double, NodeID>;
  auto cmp = [](const PQItem &a, const PQItem &b) { return a.first > b.first; };
  std::priority_queue<PQItem, std::vector<PQItem>, decltype(cmp)> pq(cmp);
  pq.emplace(0.0, startNode);

  // Dijkstra main loop
  while (!pq.empty()) {
    auto [currentDist, currentNode] = pq.top();
    pq.pop();

    if (currentDist > nodeInfo[currentNode].distance) {
      continue;
    }

    if (currentNode == targetNode) {
      break; // Early termination
    }

    // Explore neighbors
    for (const auto &[neighbor, edgeWeight] : Base::getNeighbors(currentNode)) {
      double newDistance = nodeInfo[currentNode].distance + edgeWeight;

      if (newDistance < nodeInfo[neighbor].distance) {
        nodeInfo[neighbor].distance    = newDistance;
        nodeInfo[neighbor].predecessor = currentNode;
        pq.emplace(newDistance, neighbor);
      }
    }
  }

  // Check if path was found
  if (!nodeInfo[targetNode].predecessor && startNode != targetNode) {
    return {{}, std::nullopt};
  }

  // Reconstruct path
  std::vector<NodeID> nodePath;
  NodeID              current = targetNode;
  while (current != startNode) {
    nodePath.push_back(current);
    current = *nodeInfo[current].predecessor;
  }
  nodePath.push_back(startNode);
  std::reverse(nodePath.begin(), nodePath.end());

  // Convert to path segments
  std::vector<PathSegment> pathSegments;
  for (size_t i = 0; i < nodePath.size() - 1; ++i) {
    const Pose2D &fromPose = Base::getNodePose(nodePath[i]);
    const Pose2D &toPose   = Base::getNodePose(nodePath[i + 1]);
    pathSegments.emplace_back(fromPose, toPose);
  }

  return {pathSegments, nodeInfo[targetNode].distance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
Pathfinding<NodeID, Pose2D, WeightType>::dijkstraFromPose(
    const Pose2D &startPose, const NodeID &targetNode,
    double maxDistanceToGraph) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  // Find closest node to start pose
  NodeID closestNode = Base::getClosestNode(startPose);

  // Check if distance to closest node exceeds threshold
  double distanceToClosest = Base::getDistanceToClosestNode(startPose);
  if (distanceToClosest > maxDistanceToGraph) {
    return {{}, std::nullopt}; // Pose is too far from graph
  }

  // Get path from closest node to target
  auto [nodePath, nodeDistance] = dijkstra(closestNode, targetNode);

  if (!nodeDistance) {
    return {{}, std::nullopt};
  }

  // Add initial segment from start pose to closest node
  std::vector<PathSegment> fullPath;
  const Pose2D            &closestPose     = Base::getNodePose(closestNode);
  double                   initialDistance = startPose.distanceTo(closestPose);

  if (initialDistance > 1e-6) { // Only add if not already at the node
    fullPath.emplace_back(startPose, closestPose);
  }

  // Add the rest of the path
  fullPath.insert(fullPath.end(), nodePath.begin(), nodePath.end());

  double totalDistance = initialDistance + *nodeDistance;
  return {fullPath, totalDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
Pathfinding<NodeID, Pose2D, WeightType>::dijkstraPoseToPose(
    const Pose2D &startPose, const Pose2D &targetPose,
    double maxDistanceToGraph) const {

  if (Base::empty()) {
    return {{}, std::nullopt};
  }

  // Find closest nodes
  NodeID startNode  = Base::getClosestNode(startPose);
  NodeID targetNode = Base::getClosestNode(targetPose);

  // Check if start pose is within threshold
  double startDistanceToGraph =
      startPose.distanceTo(Base::getNodePose(startNode));
  if (startDistanceToGraph > maxDistanceToGraph) {
    return {{}, std::nullopt}; // Start pose is too far from graph
  }

  // Check if target pose is within threshold
  double targetDistanceToGraph =
      targetPose.distanceTo(Base::getNodePose(targetNode));
  if (targetDistanceToGraph > maxDistanceToGraph) {
    return {{}, std::nullopt}; // Target pose is too far from graph
  }

  // Get path between nodes
  auto [nodePath, nodeDistance] = dijkstra(startNode, targetNode);

  if (!nodeDistance) {
    return {{}, std::nullopt};
  }

  // Build complete path
  std::vector<PathSegment> fullPath;
  double                   totalDistance = 0.0;

  // Start to first node
  const Pose2D &startNodePose = Base::getNodePose(startNode);
  double        startDistance = startPose.distanceTo(startNodePose);
  if (startDistance > 1e-6) {
    fullPath.emplace_back(startPose, startNodePose);
    totalDistance += startDistance;
  }

  // Node-to-node path
  fullPath.insert(fullPath.end(), nodePath.begin(), nodePath.end());
  totalDistance += *nodeDistance;

  // Last node to target
  const Pose2D &targetNodePose = Base::getNodePose(targetNode);
  double        endDistance    = targetNodePose.distanceTo(targetPose);
  if (endDistance > 1e-6) {
    fullPath.emplace_back(targetNodePose, targetPose);
    totalDistance += endDistance;
  }

  return {fullPath, totalDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
double Pathfinding<NodeID, Pose2D, WeightType>::calculatePathDistance(
    const std::vector<PathSegment> &pathSegments) const {
  double totalDistance = 0.0;
  for (const auto &segment : pathSegments) {
    totalDistance += segment.first.distanceTo(segment.second);
  }
  return totalDistance;
}

template <typename NodeID, typename Pose2D, typename WeightType>
bool Pathfinding<NodeID, Pose2D, WeightType>::isPathValid(
    const std::vector<PathSegment> &pathSegments) const {
  if (pathSegments.empty()) {
    return true;
  }

  for (size_t i = 1; i < pathSegments.size(); ++i) {
    const Pose2D &prevEnd   = pathSegments[i - 1].second;
    const Pose2D &currStart = pathSegments[i].first;

    double gap = prevEnd.distanceTo(currStart);
    if (gap > 1e-6) {
      return false;
    }
  }
  return true;
}

// Explicit instantiations for the template methods
template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<int, CPose2D, double>::dijkstra(const int &, const int &) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<unsigned long, CPose2D, double>::dijkstra(
    const unsigned long &, const unsigned long &) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<int, CPose2D, double>::dijkstraFromPose(const CPose2D &,
                                                    const int &, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<unsigned long, CPose2D, double>::dijkstraFromPose(
    const CPose2D &, const unsigned long &, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<int, CPose2D, double>::dijkstraPoseToPose(const CPose2D &,
                                                      const CPose2D &,
                                                      double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
Pathfinding<unsigned long, CPose2D, double>::dijkstraPoseToPose(const CPose2D &,
                                                                const CPose2D &,
                                                                double) const;

template double Pathfinding<int, CPose2D, double>::calculatePathDistance(
    const std::vector<std::pair<CPose2D, CPose2D>> &) const;

template double
Pathfinding<unsigned long, CPose2D, double>::calculatePathDistance(
    const std::vector<std::pair<CPose2D, CPose2D>> &) const;

template bool Pathfinding<int, CPose2D, double>::isPathValid(
    const std::vector<std::pair<CPose2D, CPose2D>> &) const;

template bool Pathfinding<unsigned long, CPose2D, double>::isPathValid(
    const std::vector<std::pair<CPose2D, CPose2D>> &) const;

} // namespace core
} // namespace mrpt_graphPose_pose