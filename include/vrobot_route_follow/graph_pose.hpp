#pragma once

// Core components
#include "core/geometric_utils.hpp"
#include "core/graph_base.hpp"
#include "core/pathfinding.hpp"

// Advanced algorithms
#include "algorithms/link_based_planner.hpp"
#include "algorithms/scoring.hpp"

// Utilities
#include "utils/nav_conversion.hpp"
#include "utils/visualization.hpp"

namespace vrobot_route_follow {

/**
 * @brief Main GraphPose class combining all functionality
 * @tparam NodeID Type for node identifiers (default: int)
 * @tparam Pose2D Type for 2D poses (default: CPose2D)
 * @tparam WeightType Type for edge weights (default: double)
 */
template <typename NodeID = int, typename Pose2D = CPose2D,
          typename WeightType = double>
class GraphPose
    : public virtual core::GraphBase<NodeID, Pose2D, WeightType>,
      public virtual core::GeometricUtils<NodeID, Pose2D, WeightType>,
      public virtual core::Pathfinding<NodeID, Pose2D, WeightType>,
      public virtual algorithms::LinkBasedPlanner<NodeID, Pose2D, WeightType>,
      public virtual algorithms::ScoringSystem<NodeID, Pose2D, WeightType>,
      public virtual utils::NavConversion<NodeID, Pose2D, WeightType>,
      public virtual utils::Visualization<NodeID, Pose2D, WeightType> {
public:
  using Base        = core::GraphBase<NodeID, Pose2D, WeightType>;
  using PathSegment = std::pair<Pose2D, Pose2D>;

  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================

  /**
   * @brief Default constructor
   */
  GraphPose() = default;

  /**
   * @brief Construct from nodes and edges
   * @param nodes Map of nodeID to pose
   * @param edges Vector of (fromID, toID, weight) tuples
   */
  GraphPose(const std::unordered_map<NodeID, Pose2D>                  &nodes,
            const std::vector<std::tuple<NodeID, NodeID, WeightType>> &edges)
      : Base(nodes, edges) {}

  // ========================================================================
  // HIGH-LEVEL PATH PLANNING INTERFACE
  // ========================================================================

  /**
   * @brief Comprehensive path planning with automatic algorithm selection
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param config Planning configuration
   * @return Planning result with path and metadata
   */
  struct PlanningConfig {
    double directThreshold = 1.5; // distance threshold for direct path
    double distanceThreshold =
        0.5;                       // distance threshold for link-based approach
    size_t maxLinks           = 4; // maximum number of links to consider
    double linkDistanceWeight = 2.0; // weight for link distance
    double maxLinkDistance =
        std::numeric_limits<double>::infinity(); // maximum distance for
                                                 // link-based approach
    double graphDistanceWeight = 1.0;            // weight for graph distance
    bool   enableLinkBased     = true;           // enable link-based approach
    bool enablePruning = false; // Disabled by default for link-based approaches
    double interpolationResolution = 0.01; // interpolation resolution for path
  };

  struct PlanningResult {
    std::vector<PathSegment>      pathSegments;
    std::optional<double>         totalDistance;
    std::string                   algorithmUsed;
    std::string                   errorMessage;
    std::map<std::string, double> metadata;
    bool                          success = false;
  };

  PlanningResult
  planPath(const Pose2D &startPose, const NodeID &targetNode,
           const PlanningConfig &config = PlanningConfig{}) const;

  /**
   * @brief Convert planning result to ROS2 nav_msgs::msg::Path
   * @param result Planning result
   * @param frameId Frame ID for the path
   * @param timestamp Timestamp for the path
   * @param resolution Interpolation resolution (meters)
   * @return ROS2 navigation path
   */
  nav_msgs::msg::Path planningResultToNavPath(const PlanningResult &result,
                                              const std::string    &frameId,
                                              const rclcpp::Time   &timestamp,
                                              double resolution = 0.01) const;

  // ========================================================================
  // DEBUGGING AND ANALYSIS
  // ========================================================================

  /**
   * @brief Debug closest links for analysis
   * @param queryPose Query pose
   * @param maxLinks Maximum number of links to analyze
   * @param maxDistance Maximum distance to consider
   */
  void debugClosestLinks(
      const Pose2D &queryPose, size_t maxLinks = 5,
      double maxDistance = std::numeric_limits<double>::infinity()) const;

  /**
   * @brief Get comprehensive graph statistics
   * @return Map of statistics
   */
  std::map<std::string, double> getGraphStatistics() const;

  // ========================================================================
  // COMPATIBILITY METHODS
  // ========================================================================

  /**
   * @brief Legacy compatibility wrapper
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  planPathToNode(const Pose2D &startPose, const NodeID &targetNode) const;
};

// ========================================================================
// TYPE ALIASES FOR CONVENIENCE
// ========================================================================

using GraphPoseInt    = GraphPose<int, CPose2D, double>;
using GraphPoseString = GraphPose<std::string, CPose2D, double>;

} // namespace vrobot_route_follow