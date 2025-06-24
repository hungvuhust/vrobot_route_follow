#pragma once

#include "geometric_utils.hpp"
#include <optional>
#include <queue>
#include <unordered_map>

namespace mrpt_graphPose_pose {
namespace core {

/**
 * @brief Core pathfinding algorithms
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class Pathfinding : public virtual GeometricUtils<NodeID, Pose2D, WeightType> {
public:
  using Base        = GeometricUtils<NodeID, Pose2D, WeightType>;
  using PathSegment = std::pair<Pose2D, Pose2D>;

  // ========================================================================
  // DIJKSTRA ALGORITHM
  // ========================================================================

  /**
   * @brief Find shortest path between two nodes using Dijkstra algorithm
   * @param startNode Starting node ID
   * @param targetNode Target node ID
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra(const NodeID &startNode, const NodeID &targetNode) const;

  /**
   * @brief Find path from arbitrary pose to target node
   * @param startPose Starting pose (not necessarily at a node)
   * @param targetNode Target node ID
   * @param maxDistanceToGraph Maximum allowed distance from pose to closest
   * node
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraFromPose(const Pose2D &startPose, const NodeID &targetNode,
                   double maxDistanceToGraph =
                       std::numeric_limits<double>::infinity()) const;

  /**
   * @brief Find path between two arbitrary poses
   * @param startPose Starting pose
   * @param targetPose Target pose
   * @param maxDistanceToGraph Maximum allowed distance from poses to closest
   * nodes
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraPoseToPose(const Pose2D &startPose, const Pose2D &targetPose,
                     double maxDistanceToGraph =
                         std::numeric_limits<double>::infinity()) const;

protected:
  // ========================================================================
  // HELPER FUNCTIONS
  // ========================================================================

  /**
   * @brief Calculate total distance of path segments
   * @param pathSegments Vector of path segments
   * @return Total distance
   */
  double
  calculatePathDistance(const std::vector<PathSegment> &pathSegments) const;

  /**
   * @brief Check if path is valid (all segments connected)
   * @param pathSegments Vector of path segments
   * @return True if path is valid
   */
  bool isPathValid(const std::vector<PathSegment> &pathSegments) const;
};

} // namespace core
} // namespace mrpt_graphPose_pose