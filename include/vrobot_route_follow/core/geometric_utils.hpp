#pragma once

#include "graph_base.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace vrobot_route_follow {
namespace core {

/**
 * @brief Geometric utility functions for graph operations
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class GeometricUtils : public virtual GraphBase<NodeID, Pose2D, WeightType> {
public:
  using Base = GraphBase<NodeID, Pose2D, WeightType>;

  // ========================================================================
  // NODE PROXIMITY OPERATIONS
  // ========================================================================

  /**
   * @brief Find nearest node to given pose
   * @param queryPose Query pose
   * @return NodeID of nearest node
   */
  NodeID getClosestNode(const Pose2D &queryPose) const;

  /**
   * @brief Get distance to closest node
   * @param queryPose Query pose
   * @return Distance to nearest node
   */
  double getDistanceToClosestNode(const Pose2D &queryPose) const;

  // ========================================================================
  // LINK GEOMETRY OPERATIONS
  // ========================================================================

  /**
   * @brief Calculate distance from pose to line segment (link)
   * @param queryPose Query pose
   * @param nodeA First node of the link
   * @param nodeB Second node of the link
   * @return Distance from pose to line segment
   */
  double distanceToLink(const Pose2D &queryPose, const NodeID &nodeA,
                        const NodeID &nodeB) const;

  /**
   * @brief Find projection point of pose onto line segment
   * @param queryPose Query pose
   * @param nodeA First node of the link
   * @param nodeB Second node of the link
   * @return Projection point on line segment
   */
  Pose2D projectOntoLink(const Pose2D &queryPose, const NodeID &nodeA,
                         const NodeID &nodeB) const;

  /**
   * @brief Find closest links to given pose with distance filtering
   * @param queryPose Query pose
   * @param maxLinks Maximum number of links to return
   * @param maxDistance Maximum distance threshold for links
   * @return Vector of (nodeA, nodeB, distance) sorted by distance
   */
  std::vector<std::tuple<NodeID, NodeID, double>> getClosestLinks(
      const Pose2D &queryPose, size_t maxLinks = 3,
      double maxDistance = std::numeric_limits<double>::infinity()) const;

private:
  // ========================================================================
  // INTERNAL GEOMETRIC CALCULATIONS
  // ========================================================================

  /**
   * @brief Calculate distance from point to line segment
   * @param point Query point
   * @param lineStart Start point of line segment
   * @param lineEnd End point of line segment
   * @return Distance from point to line segment
   */
  double calculatePointToLineSegmentDistance(const Pose2D &point,
                                             const Pose2D &lineStart,
                                             const Pose2D &lineEnd) const;

  /**
   * @brief Calculate projection of point onto line segment
   * @param point Query point
   * @param lineStart Start point of line segment
   * @param lineEnd End point of line segment
   * @return Projection point on line segment
   */
  Pose2D calculateProjectionOnLineSegment(const Pose2D &point,
                                          const Pose2D &lineStart,
                                          const Pose2D &lineEnd) const;
};

} // namespace core
} // namespace vrobot_route_follow