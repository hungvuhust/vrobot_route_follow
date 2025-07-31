#pragma once

#include "link_info.hpp"
#include "curve_link_info.hpp"
#include "node_info.hpp"
#include <Eigen/Dense>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <vrobot_local_planner/msg/path.hpp>

// Forward declaration to avoid circular dependency
namespace vrobot_route_follow {
namespace data_structures {
  struct EnhancedPathSegment;
}
}

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Rich path result with full traceability
 *
 * This structure contains complete information about the planned path,
 * including the sequence of nodes and links traversed, making it easy
 * to trace and debug the path planning process.
 */
struct RichPathResult {
  // ========================================================================
  // CORE PATH DATA
  // ========================================================================

  /// Sequence of nodes in the path (including start and end)
  std::vector<NodeInfo> nodeSequence;

  /// Sequence of links connecting the nodes
  std::vector<LinkInfo> linkSequence;

  /// Sequence of curved links (if any) in the path
  std::vector<CurveLinkInfo> curvedLinkSequence;

  /// Interpolated poses along the path for smooth navigation
  std::vector<Eigen::Vector3d> poseSequence;

  /// Velocity profile for each pose (optional)
  std::vector<double> velocityProfile;

  // ========================================================================
  // PATH METRICS
  // ========================================================================

  /// Total distance of the path
  double totalDistance = 0.0;

  /// Estimated total time (if velocity profile is available)
  std::optional<double> totalTime;

  /// Maximum velocity along the path
  std::optional<double> maxVelocity;

  /// Minimum velocity along the path
  std::optional<double> minVelocity;

  // ========================================================================
  // PLANNING METADATA
  // ========================================================================

  /// Algorithm used for path planning
  std::string algorithmUsed;

  /// Success flag
  bool success = false;

  /// Error message if planning failed
  std::string errorMessage;

  /// Additional metadata from the planning process
  std::map<std::string, double> metadata;

  /// Planning time in milliseconds
  std::optional<double> planningTimeMs;

  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================

  /**
   * @brief Default constructor
   */
  RichPathResult() = default;

  /**
   * @brief Constructor for successful path
   */
  RichPathResult(const std::vector<NodeInfo> &nodes,
                 const std::vector<LinkInfo> &links,
                 const std::string           &algorithm)
      : nodeSequence(nodes), linkSequence(links), algorithmUsed(algorithm),
        success(true) {
    calculateMetrics();
  }

  /**
   * @brief Constructor for failed path
   */
  RichPathResult(const std::string &algorithm, const std::string &error)
      : algorithmUsed(algorithm), success(false), errorMessage(error) {}

  // ========================================================================
  // METRIC CALCULATION
  // ========================================================================

  /**
   * @brief Calculate path metrics (distance, time, etc.)
   */
  void calculateMetrics() {
    if (nodeSequence.size() < 2) {
      totalDistance = 0.0;
      return;
    }

    totalDistance        = 0.0;
    double totalTimeCalc = 0.0;
    double maxVel        = 0.0;
    double minVel        = std::numeric_limits<double>::max();

    for (size_t i = 0; i < linkSequence.size(); ++i) {
      const auto &link      = linkSequence[i];
      const auto &startNode = nodeSequence[i];
      const auto &endNode   = nodeSequence[i + 1];

      double segmentDistance = link.calculateWeight(startNode, endNode);
      totalDistance += segmentDistance;

      if (link.max_velocity > 0) {
        double segmentTime = segmentDistance / link.max_velocity;
        totalTimeCalc += segmentTime;
        maxVel = std::max(maxVel, link.max_velocity);
        minVel = std::min(minVel, link.max_velocity);
      }
    }

    if (maxVel > 0) {
      totalTime   = totalTimeCalc;
      maxVelocity = maxVel;
      minVelocity =
          (minVel == std::numeric_limits<double>::max()) ? maxVel : minVel;
    }
  }

  /**
   * @brief Generate interpolated poses along the path
   */
  void generatePoseSequence(double resolution = 0.02) {
    poseSequence.clear();
    velocityProfile.clear();

    if (nodeSequence.empty())
      return;

    // Add start pose
    poseSequence.push_back(nodeSequence[0].toPose());

    for (size_t i = 0; i < linkSequence.size(); ++i) {
      const auto &startNode = nodeSequence[i];
      const auto &endNode   = nodeSequence[i + 1];
      const auto &link      = linkSequence[i];

      // Interpolate between start and end
      Eigen::Vector3d startPose = startNode.toPose();
      Eigen::Vector3d endPose   = endNode.toPose();

      double distance = startNode.distanceTo(endNode);
      int    numSteps = std::max(1, static_cast<int>(distance / resolution));

      for (int step = 1; step <= numSteps; ++step) {
        double          t = static_cast<double>(step) / numSteps;
        Eigen::Vector3d interpolatedPose =
            startPose + t * (endPose - startPose);
        poseSequence.push_back(interpolatedPose);

        // Add velocity if available
        if (link.max_velocity > 0) {
          velocityProfile.push_back(link.max_velocity);
        }
      }
    }
  }

  // ========================================================================
  // CONVERSION METHODS
  // ========================================================================

  /**
   * @brief Convert to legacy PathSegment format
   */
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  toPathSegments() const {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments;

    for (size_t i = 0; i < nodeSequence.size() - 1; ++i) {
      segments.emplace_back(nodeSequence[i].toPose(),
                            nodeSequence[i + 1].toPose());
    }

    return segments;
  }

  /**
   * @brief Convert to ROS nav_msgs::Path
   */
  nav_msgs::msg::Path toNavPath(const std::string  &frame_id,
                                const rclcpp::Time &timestamp) const;

  /**
   * @brief Convert to vrobot_local_planner::Path
   * NOTE: Implementation moved to VPathBuilder for better flexibility
   */
  vrobot_local_planner::msg::Path toVPath(const std::string  &frame_id,
                                          const rclcpp::Time &timestamp) const;

  /**
   * @brief Convert to enhanced path segments with rich attributes
   */
  std::vector<data_structures::EnhancedPathSegment> toEnhancedSegments() const;

  // ========================================================================
  // ANALYSIS METHODS
  // ========================================================================

  /**
   * @brief Get path summary string
   */
  std::string getSummary() const {
    if (!success) {
      return "FAILED: " + errorMessage + " (Algorithm: " + algorithmUsed + ")";
    }

    std::string summary = "SUCCESS: " + algorithmUsed +
                          " | Nodes: " + std::to_string(nodeSequence.size()) +
                          " | Links: " + std::to_string(linkSequence.size()) +
                          " | Distance: " + std::to_string(totalDistance) + "m";

    if (totalTime.has_value()) {
      summary += " | Time: " + std::to_string(totalTime.value()) + "s";
    }

    if (planningTimeMs.has_value()) {
      summary +=
          " | Planning: " + std::to_string(planningTimeMs.value()) + "ms";
    }

    return summary;
  }

  /**
   * @brief Get detailed path trace
   */
  std::string getDetailedTrace() const {
    if (!success) {
      return getSummary();
    }

    std::string trace = "Path Trace:\n";

    for (size_t i = 0; i < nodeSequence.size(); ++i) {
      const auto &node = nodeSequence[i];
      trace += "  Node " + std::to_string(i) + ": " + node.toString() + "\n";

      if (i < linkSequence.size()) {
        const auto &link = linkSequence[i];
        trace += "    -> Link: " + link.toString() + "\n";
      }
    }

    return trace;
  }

  /**
   * @brief Get node names sequence for easy reading
   */
  std::vector<std::string> getNodeNameSequence() const {
    std::vector<std::string> names;
    for (const auto &node : nodeSequence) {
      names.push_back(node.toShortString());
    }
    return names;
  }

  /**
   * @brief Get link IDs sequence
   */
  std::vector<int32_t> getLinkIdSequence() const {
    std::vector<int32_t> ids;
    for (const auto &link : linkSequence) {
      ids.push_back(link.id_straight_link);
    }
    return ids;
  }

  // ========================================================================
  // VALIDATION
  // ========================================================================

  /**
   * @brief Validate path consistency
   */
  bool validate() const {
    if (!success)
      return false;

    if (nodeSequence.size() < 2)
      return false;

    if (linkSequence.size() != nodeSequence.size() - 1)
      return false;

    // Check link connectivity
    for (size_t i = 0; i < linkSequence.size(); ++i) {
      const auto &link      = linkSequence[i];
      const auto &startNode = nodeSequence[i];
      const auto &endNode   = nodeSequence[i + 1];

      if (link.id_start != startNode.id || link.id_end != endNode.id) {
        return false;
      }
    }

    return true;
  }

  // ========================================================================
  // SERIALIZATION
  // ========================================================================

  /**
   * @brief Convert to JSON string
   */
  std::string toJsonString() const;

  /**
   * @brief Create from JSON string
   */
  static RichPathResult fromJsonString(const std::string &json_str);
};

} // namespace data_structures
} // namespace vrobot_route_follow