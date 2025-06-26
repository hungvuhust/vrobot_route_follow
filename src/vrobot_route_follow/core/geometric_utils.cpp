#include "vrobot_route_follow/core/geometric_utils.hpp"

namespace vrobot_route_follow {
namespace core {

// Explicit template instantiations for common types
template class GeometricUtils<int, CPose2D, double>;
template class GeometricUtils<long, CPose2D, double>;
template class GeometricUtils<unsigned long, CPose2D, double>;

// Implementation for commonly used methods can be added here
// Currently, most methods are inline in the header

template <typename NodeID, typename Pose2D, typename WeightType>
NodeID GeometricUtils<NodeID, Pose2D, WeightType>::getClosestNode(
    const Pose2D &queryPose) const {
  if (Base::nodePoses_.empty()) {
    throw std::runtime_error("Graph is empty");
  }

  NodeID bestNode;
  double minDist = std::numeric_limits<double>::max();

  for (const auto &kv : Base::nodePoses_) {
    double dist = kv.second.distanceTo(queryPose);
    if (dist < minDist) {
      minDist  = dist;
      bestNode = kv.first;
    }
  }
  return bestNode;
}

template <typename NodeID, typename Pose2D, typename WeightType>
double GeometricUtils<NodeID, Pose2D, WeightType>::getDistanceToClosestNode(
    const Pose2D &queryPose) const {
  NodeID closestNode = getClosestNode(queryPose);
  return Base::nodePoses_.at(closestNode).distanceTo(queryPose);
}

template <typename NodeID, typename Pose2D, typename WeightType>
double GeometricUtils<NodeID, Pose2D, WeightType>::distanceToLink(
    const Pose2D &queryPose, const NodeID &nodeA, const NodeID &nodeB) const {
  if (!Base::hasNode(nodeA) || !Base::hasNode(nodeB)) {
    return std::numeric_limits<double>::max();
  }

  const auto &poseA = Base::getNodePose(nodeA);
  const auto &poseB = Base::getNodePose(nodeB);

  return calculatePointToLineSegmentDistance(queryPose, poseA, poseB);
}

template <typename NodeID, typename Pose2D, typename WeightType>
Pose2D GeometricUtils<NodeID, Pose2D, WeightType>::projectOntoLink(
    const Pose2D &queryPose, const NodeID &nodeA, const NodeID &nodeB) const {
  if (!Base::hasNode(nodeA) || !Base::hasNode(nodeB)) {
    throw std::runtime_error("Invalid node IDs for projection");
  }

  const auto &poseA = Base::getNodePose(nodeA);
  const auto &poseB = Base::getNodePose(nodeB);

  return calculateProjectionOnLineSegment(queryPose, poseA, poseB);
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::vector<std::tuple<NodeID, NodeID, double>>
GeometricUtils<NodeID, Pose2D, WeightType>::getClosestLinks(
    const Pose2D &queryPose, size_t maxLinks, double maxDistance) const {

  std::vector<std::tuple<NodeID, NodeID, double>> linkDistances;

  // Evaluate all directed links in the graph
  for (const auto &nodeKV : Base::adjList_) {
    NodeID fromNode = nodeKV.first;
    for (const auto &edge : nodeKV.second) {
      NodeID toNode = edge.first;
      double dist   = distanceToLink(queryPose, fromNode, toNode);

      if (dist <= maxDistance) {
        linkDistances.emplace_back(fromNode, toNode, dist);
      }
    }
  }

  // Sort by distance and limit results
  std::sort(linkDistances.begin(), linkDistances.end(),
            [](const auto &a, const auto &b) {
              return std::get<2>(a) < std::get<2>(b);
            });

  if (linkDistances.size() > maxLinks) {
    linkDistances.resize(maxLinks);
  }

  return linkDistances;
}

template <typename NodeID, typename Pose2D, typename WeightType>
double
GeometricUtils<NodeID, Pose2D, WeightType>::calculatePointToLineSegmentDistance(
    const Pose2D &point, const Pose2D &lineStart, const Pose2D &lineEnd) const {
  double dx = lineEnd.x() - lineStart.x();
  double dy = lineEnd.y() - lineStart.y();
  double px = point.x() - lineStart.x();
  double py = point.y() - lineStart.y();

  double lengthSq = dx * dx + dy * dy;
  if (lengthSq == 0) {
    return lineStart.distanceTo(point);
  }

  double t = std::max(0.0, std::min(1.0, (px * dx + py * dy) / lengthSq));
  double closestX = lineStart.x() + t * dx;
  double closestY = lineStart.y() + t * dy;
  double distX    = point.x() - closestX;
  double distY    = point.y() - closestY;

  return std::sqrt(distX * distX + distY * distY);
}

template <typename NodeID, typename Pose2D, typename WeightType>
Pose2D
GeometricUtils<NodeID, Pose2D, WeightType>::calculateProjectionOnLineSegment(
    const Pose2D &point, const Pose2D &lineStart, const Pose2D &lineEnd) const {
  double dx = lineEnd.x() - lineStart.x();
  double dy = lineEnd.y() - lineStart.y();
  double px = point.x() - lineStart.x();
  double py = point.y() - lineStart.y();

  double lengthSq = dx * dx + dy * dy;
  if (lengthSq == 0) {
    return lineStart;
  }

  double t = std::max(0.0, std::min(1.0, (px * dx + py * dy) / lengthSq));
  double closestX = lineStart.x() + t * dx;
  double closestY = lineStart.y() + t * dy;
  double theta    = lineStart.phi() + t * (lineEnd.phi() - lineStart.phi());

  return Pose2D(closestX, closestY, theta);
}

// Explicit instantiations for the template methods
template int
GeometricUtils<int, CPose2D, double>::getClosestNode(const CPose2D &) const;
template double GeometricUtils<int, CPose2D, double>::getDistanceToClosestNode(
    const CPose2D &) const;
template double GeometricUtils<int, CPose2D, double>::distanceToLink(
    const CPose2D &, const int &, const int &) const;
template CPose2D GeometricUtils<int, CPose2D, double>::projectOntoLink(
    const CPose2D &, const int &, const int &) const;
template std::vector<std::tuple<int, int, double>>
GeometricUtils<int, CPose2D, double>::getClosestLinks(const CPose2D &, size_t,
                                                      double) const;

template unsigned long
GeometricUtils<unsigned long, CPose2D, double>::getClosestNode(
    const CPose2D &) const;
template double
GeometricUtils<unsigned long, CPose2D, double>::getDistanceToClosestNode(
    const CPose2D &) const;
template double GeometricUtils<unsigned long, CPose2D, double>::distanceToLink(
    const CPose2D &, const unsigned long &, const unsigned long &) const;
template CPose2D
GeometricUtils<unsigned long, CPose2D, double>::projectOntoLink(
    const CPose2D &, const unsigned long &, const unsigned long &) const;
template std::vector<std::tuple<unsigned long, unsigned long, double>>
GeometricUtils<unsigned long, CPose2D, double>::getClosestLinks(const CPose2D &,
                                                                size_t,
                                                                double) const;

} // namespace core
} // namespace vrobot_route_follow