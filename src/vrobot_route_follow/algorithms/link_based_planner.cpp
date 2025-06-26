#include "vrobot_route_follow/algorithms/link_based_planner.hpp"
#include <limits>

namespace vrobot_route_follow {
namespace algorithms {

// Explicit template instantiations for common types
template class LinkBasedPlanner<int, CPose2D, double>;
template class LinkBasedPlanner<long, CPose2D, double>;
template class LinkBasedPlanner<unsigned long, CPose2D, double>;

// Implementation for commonly used methods can be added here
// Currently, most methods are inline in the header

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::dijkstraWithModularLinkApproach(
    const Pose2D &startPose, const NodeID &targetNode, double directThreshold,
    size_t maxLinks, double linkDistanceWeight, double maxLinkDistance,
    double graphDistanceWeight) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  const Pose2D &targetPose = Base::getNodePose(targetNode);

  // Check for direct path first
  double directDistance = startPose.distanceTo(targetPose);
  if (directDistance <= directThreshold) {
    std::vector<PathSegment> directPath = {
        {startPose, targetPose}
    };
    return {directPath, directDistance};
  }

  // Step 1: Find closest links with distance filtering
  auto closestLinks =
      Base::getClosestLinks(startPose, maxLinks, maxLinkDistance);
  if (closestLinks.empty()) {
    return Base::dijkstraFromPose(startPose, targetNode, maxLinkDistance);
  }

  // Step 2: Evaluate each link using weighted scoring
  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestScore;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Calculate path from link to destination
    auto [linkToTarget, linkToTargetDist] = Base::dijkstra(linkEnd, targetNode);
    if (!linkToTargetDist) {
      continue;
    }

    // Calculate weighted score
    double score =
        this->calculateWeightedScore(linkDistance, *linkToTargetDist,
                                     linkDistanceWeight, graphDistanceWeight);

    if (!bestScore || score < *bestScore) {
      // Step 3: Build complete path
      Pose2D projectionPoint =
          Base::projectOntoLink(startPose, linkStart, linkEnd);
      const Pose2D &linkEndPose = Base::getNodePose(linkEnd);

      std::vector<PathSegment> candidatePath;
      candidatePath.emplace_back(startPose, projectionPoint);
      candidatePath.emplace_back(projectionPoint, linkEndPose);
      candidatePath.insert(candidatePath.end(), linkToTarget.begin(),
                           linkToTarget.end());

      bestPath  = candidatePath;
      bestScore = score;
    }
  }

  if (!bestPath) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  // Calculate actual total distance
  double totalDistance = Base::calculatePathDistance(*bestPath);
  return {*bestPath, totalDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::dijkstraWithLinkAccess(
    const Pose2D &startPose, const NodeID &targetNode, double distanceThreshold,
    size_t maxLinks, double linkDistanceWeight) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  // Check if close enough to use standard approach
  double distanceToClosest = Base::getDistanceToClosestNode(startPose);
  if (distanceToClosest <= distanceThreshold) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  // Use link-based approach for distant poses
  return dijkstraWithModularLinkApproach(
      startPose, targetNode, distanceThreshold, maxLinks, linkDistanceWeight);
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::dijkstraWithSimpleLinkAccess(
    const Pose2D &startPose, const NodeID &targetNode, size_t maxLinks) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  // Find closest links
  auto closestLinks = Base::getClosestLinks(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestDistance;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Project onto link
    Pose2D projectionPoint =
        Base::projectOntoLink(startPose, linkStart, linkEnd);

    // Find closer node on the link
    const Pose2D &startNodePose = Base::getNodePose(linkStart);
    const Pose2D &endNodePose   = Base::getNodePose(linkEnd);

    NodeID accessNode = (projectionPoint.distanceTo(startNodePose) <
                         projectionPoint.distanceTo(endNodePose))
                            ? linkStart
                            : linkEnd;

    // Path from access node to target
    auto [nodeToTarget, nodeToTargetDist] =
        Base::dijkstra(accessNode, targetNode);
    if (!nodeToTargetDist) {
      continue;
    }

    // Build complete path
    std::vector<PathSegment> candidatePath;
    candidatePath.emplace_back(startPose, projectionPoint);
    candidatePath.emplace_back(projectionPoint, Base::getNodePose(accessNode));
    candidatePath.insert(candidatePath.end(), nodeToTarget.begin(),
                         nodeToTarget.end());

    double totalDistance = Base::calculatePathDistance(candidatePath);

    if (!bestDistance || totalDistance < *bestDistance) {
      bestPath     = candidatePath;
      bestDistance = totalDistance;
    }
  }

  if (!bestPath) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  return {*bestPath, *bestDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::dijkstraWithLinkFollowing(
    const Pose2D &startPose, const NodeID &targetNode, size_t maxLinks) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  auto closestLinks = Base::getClosestLinks(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestDistance;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Try both directions on the link
    auto result1 =
        buildPathThroughLink(startPose, linkStart, linkEnd, targetNode);
    auto result2 =
        buildPathThroughLink(startPose, linkEnd, linkStart, targetNode);

    for (const auto &[path, distance] : {result1, result2}) {
      if (distance && (!bestDistance || *distance < *bestDistance)) {
        bestPath     = path;
        bestDistance = distance;
      }
    }
  }

  if (!bestPath) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  return {*bestPath, *bestDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::dijkstraWithSmartLinkFollowing(
    const Pose2D &startPose, const NodeID &targetNode, size_t maxLinks,
    double adaptiveWeight) const {

  if (!Base::hasNode(targetNode)) {
    return {{}, std::nullopt};
  }

  auto closestLinks = Base::getClosestLinks(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestScore;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    auto [path, distance] =
        buildPathThroughLink(startPose, linkStart, linkEnd, targetNode);
    if (!distance)
      continue;

    // Calculate adaptive score
    double score = linkDistance * adaptiveWeight + *distance;

    if (!bestScore || score < *bestScore) {
      bestPath  = path;
      bestScore = score;
    }
  }

  if (!bestPath) {
    return Base::dijkstraFromPose(startPose, targetNode);
  }

  double totalDistance = Base::calculatePathDistance(*bestPath);
  return {*bestPath, totalDistance};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<std::vector<std::pair<Pose2D, Pose2D>>, std::optional<double>>
LinkBasedPlanner<NodeID, Pose2D, WeightType>::buildPathThroughLink(
    const Pose2D &startPose, const NodeID &linkStart, const NodeID &linkEnd,
    const NodeID &targetNode) const {

  // Path from exit node to target
  auto [exitToTarget, exitToTargetDist] = Base::dijkstra(linkEnd, targetNode);
  if (!exitToTargetDist) {
    return {{}, std::nullopt};
  }

  // Build complete path
  Pose2D projectionPoint = Base::projectOntoLink(startPose, linkStart, linkEnd);
  const Pose2D &accessPose = Base::getNodePose(linkStart);
  const Pose2D &exitPose   = Base::getNodePose(linkEnd);

  std::vector<PathSegment> completePath = {
      {      startPose, projectionPoint}, // Approach to link
      {projectionPoint,      accessPose}, // Move to link start
      {     accessPose,        exitPose}, // Follow link topology
  };
  completePath.insert(completePath.end(), exitToTarget.begin(),
                      exitToTarget.end());

  double totalDistance = Base::calculatePathDistance(completePath);
  return {completePath, totalDistance};
}

// Explicit instantiations for the template methods
template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::dijkstraWithModularLinkApproach(
    const CPose2D &, const int &, double, size_t, double, double, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D,
                 double>::dijkstraWithModularLinkApproach(const CPose2D &,
                                                          const unsigned long &,
                                                          double, size_t,
                                                          double, double,
                                                          double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::dijkstraWithLinkAccess(const CPose2D &,
                                                               const int &,
                                                               double, size_t,
                                                               double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D, double>::dijkstraWithLinkAccess(
    const CPose2D &, const unsigned long &, double, size_t, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::dijkstraWithSimpleLinkAccess(
    const CPose2D &, const int &, size_t) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D, double>::dijkstraWithSimpleLinkAccess(
    const CPose2D &, const unsigned long &, size_t) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::dijkstraWithLinkFollowing(
    const CPose2D &, const int &, size_t) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D, double>::dijkstraWithLinkFollowing(
    const CPose2D &, const unsigned long &, size_t) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::dijkstraWithSmartLinkFollowing(
    const CPose2D &, const int &, size_t, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D,
                 double>::dijkstraWithSmartLinkFollowing(const CPose2D &,
                                                         const unsigned long &,
                                                         size_t, double) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<int, CPose2D, double>::buildPathThroughLink(const CPose2D &,
                                                             const int &,
                                                             const int &,
                                                             const int &) const;

template std::pair<std::vector<std::pair<CPose2D, CPose2D>>,
                   std::optional<double>>
LinkBasedPlanner<unsigned long, CPose2D, double>::buildPathThroughLink(
    const CPose2D &, const unsigned long &, const unsigned long &,
    const unsigned long &) const;

} // namespace algorithms
} // namespace vrobot_route_follow