#pragma once

#include "../core/pathfinding.hpp"
#include "scoring.hpp"

namespace vrobot_route_follow {
namespace algorithms {

/**
 * @brief Link-based path planning algorithms
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class LinkBasedPlanner
    : public virtual core::Pathfinding<NodeID, Pose2D, WeightType>,
      public virtual ScoringSystem<NodeID, Pose2D, WeightType> {
public:
  using Base        = core::Pathfinding<NodeID, Pose2D, WeightType>;
  using PathSegment = std::pair<Pose2D, Pose2D>;

  // ========================================================================
  // CORE LINK-BASED ALGORITHMS
  // ========================================================================

  /**
   * @brief Advanced path planning with modular link-based approach
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param directThreshold Distance threshold for direct path
   * @param maxLinks Maximum number of links to evaluate
   * @param linkDistanceWeight Weight for link distance in scoring
   * @param maxLinkDistance Maximum distance to consider links
   * @param graphDistanceWeight Weight for graph distance in scoring
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraWithModularLinkApproach(
      const Pose2D &startPose, const NodeID &targetNode,
      double directThreshold = 1.5, size_t maxLinks = 3,
      double linkDistanceWeight  = 2.0,
      double maxLinkDistance     = std::numeric_limits<double>::infinity(),
      double graphDistanceWeight = 1.0) const;

  // ========================================================================
  // ENHANCED LINK ACCESS
  // ========================================================================

  /**
   * @brief Enhanced path planning with link access for distant poses
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param distanceThreshold Threshold for using link-based approach
   * @param maxLinks Maximum number of links to consider
   * @param linkDistanceWeight Weight for link distance scoring
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraWithLinkAccess(const Pose2D &startPose, const NodeID &targetNode,
                         double distanceThreshold = 0.5, size_t maxLinks = 3,
                         double linkDistanceWeight = 2.0) const;

  // ========================================================================
  // SIMPLE LINK ACCESS
  // ========================================================================

  /**
   * @brief Simple link access approach: start → closest point on link →
   * destination
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraWithSimpleLinkAccess(const Pose2D &startPose,
                               const NodeID &targetNode,
                               size_t        maxLinks = 3) const;

  // ========================================================================
  // LINK FOLLOWING APPROACHES
  // ========================================================================

  /**
   * @brief Link following approach where robot follows link topology
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraWithLinkFollowing(const Pose2D &startPose, const NodeID &targetNode,
                            size_t maxLinks = 3) const;

  /**
   * @brief Smart link following with adaptive path selection
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @param adaptiveWeight Adaptive weight factor
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstraWithSmartLinkFollowing(const Pose2D &startPose,
                                 const NodeID &targetNode, size_t maxLinks = 3,
                                 double adaptiveWeight = 1.5) const;

private:
  // ========================================================================
  // HELPER METHODS
  // ========================================================================

  /**
   * @brief Helper method to build path through link
   * @param startPose Starting pose
   * @param linkStart Start node of link
   * @param linkEnd End node of link
   * @param targetNode Target node
   * @return Path segments and distance
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  buildPathThroughLink(const Pose2D &startPose, const NodeID &linkStart,
                       const NodeID &linkEnd, const NodeID &targetNode) const;
};

} // namespace algorithms
} // namespace vrobot_route_follow