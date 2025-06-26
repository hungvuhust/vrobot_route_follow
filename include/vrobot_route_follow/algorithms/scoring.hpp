#pragma once

#include "../core/graph_base.hpp"

namespace vrobot_route_follow {
namespace algorithms {

/**
 * @brief Scoring system for path evaluation and optimization
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses
 * @tparam WeightType Type for edge weights
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class ScoringSystem
    : public virtual core::GraphBase<NodeID, Pose2D, WeightType> {
public:
  using Base = core::GraphBase<NodeID, Pose2D, WeightType>;

  // ========================================================================
  // WEIGHTED SCORING FUNCTIONS
  // ========================================================================

  /**
   * @brief Calculate weighted score for link-based path planning
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph from link to target
   * @param linkWeight Weight for link distance component
   * @param graphWeight Weight for graph distance component
   * @return Weighted score (lower is better)
   */
  double calculateWeightedScore(double linkDistance, double graphDistance,
                                double linkWeight  = 2.0,
                                double graphWeight = 1.0) const;

  /**
   * @brief Calculate normalized weighted score
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph from link to target
   * @param linkWeight Weight for link distance component
   * @param graphWeight Weight for graph distance component
   * @param maxLinkDistance Maximum link distance for normalization
   * @param maxGraphDistance Maximum graph distance for normalization
   * @return Normalized weighted score (0-1 range, lower is better)
   */
  double calculateNormalizedWeightedScore(double linkDistance,
                                          double graphDistance,
                                          double linkWeight       = 2.0,
                                          double graphWeight      = 1.0,
                                          double maxLinkDistance  = 1.0,
                                          double maxGraphDistance = 10.0) const;

  // ========================================================================
  // MULTI-CRITERIA SCORING
  // ========================================================================

  /**
   * @brief Multi-criteria scoring for comprehensive path evaluation
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph
   * @param pathComplexity Number of segments in path
   * @param linkAngle Angle alignment with desired direction
   * @param weights Vector of weights [link, graph, complexity, angle]
   * @return Multi-criteria score
   */
  double calculateMultiCriteriaScore(double linkDistance, double graphDistance,
                                     int pathComplexity, double linkAngle,
                                     const std::vector<double> &weights = {
                                         2.0, 1.0, 0.5, 0.3}) const;

  // ========================================================================
  // ADAPTIVE SCORING
  // ========================================================================

  /**
   * @brief Adaptive scoring that adjusts weights based on context
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph
   * @param robotSpeed Current robot speed (affects time preferences)
   * @param batteryLevel Robot battery level (affects efficiency preferences)
   * @return Context-adaptive score
   */
  double calculateAdaptiveScore(double linkDistance, double graphDistance,
                                double robotSpeed   = 1.0,
                                double batteryLevel = 1.0) const;

  // ========================================================================
  // SCORING ANALYSIS
  // ========================================================================

  /**
   * @brief Analyze and compare multiple scoring options
   * @param linkDistances Vector of link distances
   * @param graphDistances Vector of corresponding graph distances
   * @param linkWeight Weight for link distance
   * @param graphWeight Weight for graph distance
   * @return Vector of scores with analysis
   */
  std::vector<std::pair<double, std::string>>
  analyzeScoring(const std::vector<double> &linkDistances,
                 const std::vector<double> &graphDistances,
                 double linkWeight = 2.0, double graphWeight = 1.0) const;

  /**
   * @brief Get optimal weights through simple optimization
   * @param trainingData Vector of (linkDist, graphDist, optimalChoice) tuples
   * @return Optimal weights [linkWeight, graphWeight]
   */
  std::pair<double, double> optimizeWeights(
      const std::vector<std::tuple<double, double, bool>> &trainingData) const;

  // ========================================================================
  // EVALUATION METRICS
  // ========================================================================

  /**
   * @brief Calculate evaluation metrics for scoring performance
   * @param actualChoices Actual optimal choices (ground truth)
   * @param predictedChoices Predicted choices from scoring
   * @return Tuple of (accuracy, precision, recall)
   */
  std::tuple<double, double, double>
  calculateMetrics(const std::vector<bool> &actualChoices,
                   const std::vector<bool> &predictedChoices) const;
};

} // namespace algorithms
} // namespace vrobot_route_follow