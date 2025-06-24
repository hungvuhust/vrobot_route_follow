#include "vrobot_route_follow/algorithms/scoring.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mrpt_graphPose_pose {
namespace algorithms {

// Explicit template instantiations for common types
template class ScoringSystem<int, CPose2D, double>;
template class ScoringSystem<long, CPose2D, double>;
template class ScoringSystem<unsigned long, CPose2D, double>;

template <typename NodeID, typename Pose2D, typename WeightType>
double ScoringSystem<NodeID, Pose2D, WeightType>::calculateWeightedScore(
    double linkDistance, double graphDistance, double linkWeight,
    double graphWeight) const {
  return (linkDistance * linkWeight) + (graphDistance * graphWeight);
}

template <typename NodeID, typename Pose2D, typename WeightType>
double
ScoringSystem<NodeID, Pose2D, WeightType>::calculateNormalizedWeightedScore(
    double linkDistance, double graphDistance, double linkWeight,
    double graphWeight, double maxLinkDistance, double maxGraphDistance) const {

  double normalizedLinkDist  = std::min(linkDistance / maxLinkDistance, 1.0);
  double normalizedGraphDist = std::min(graphDistance / maxGraphDistance, 1.0);

  double totalWeight = linkWeight + graphWeight;
  return ((normalizedLinkDist * linkWeight) +
          (normalizedGraphDist * graphWeight)) /
         totalWeight;
}

template <typename NodeID, typename Pose2D, typename WeightType>
double ScoringSystem<NodeID, Pose2D, WeightType>::calculateMultiCriteriaScore(
    double linkDistance, double graphDistance, int pathComplexity,
    double linkAngle, const std::vector<double> &weights) const {

  if (weights.size() != 4) {
    throw std::invalid_argument("Weights vector must have 4 elements");
  }

  // Normalize components
  double normalizedLinkDist   = std::min(linkDistance / 2.0, 1.0);
  double normalizedGraphDist  = std::min(graphDistance / 10.0, 1.0);
  double normalizedComplexity = std::min(pathComplexity / 10.0, 1.0);
  double normalizedAngle      = std::abs(linkAngle) / M_PI; // 0-1 range

  return (normalizedLinkDist * weights[0]) +
         (normalizedGraphDist * weights[1]) +
         (normalizedComplexity * weights[2]) + (normalizedAngle * weights[3]);
}

template <typename NodeID, typename Pose2D, typename WeightType>
double ScoringSystem<NodeID, Pose2D, WeightType>::calculateAdaptiveScore(
    double linkDistance, double graphDistance, double robotSpeed,
    double batteryLevel) const {

  // Adapt weights based on context
  double linkWeight  = 2.0;
  double graphWeight = 1.0;

  // If robot is slow, prefer shorter link access distances
  if (robotSpeed < 0.5) {
    linkWeight *= 1.5;
  }

  // If battery is low, prefer shorter overall paths
  if (batteryLevel < 0.3) {
    graphWeight *= 1.8;
  }

  return calculateWeightedScore(linkDistance, graphDistance, linkWeight,
                                graphWeight);
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::vector<std::pair<double, std::string>>
ScoringSystem<NodeID, Pose2D, WeightType>::analyzeScoring(
    const std::vector<double> &linkDistances,
    const std::vector<double> &graphDistances, double linkWeight,
    double graphWeight) const {

  if (linkDistances.size() != graphDistances.size()) {
    throw std::invalid_argument(
        "Link and graph distance vectors must have same size");
  }

  std::vector<std::pair<double, std::string>> results;

  for (size_t i = 0; i < linkDistances.size(); ++i) {
    double score = calculateWeightedScore(linkDistances[i], graphDistances[i],
                                          linkWeight, graphWeight);

    std::string analysis = "Link" + std::to_string(i) +
                           ": linkDist=" + std::to_string(linkDistances[i]) +
                           ", graphDist=" + std::to_string(graphDistances[i]) +
                           ", score=" + std::to_string(score);

    results.emplace_back(score, analysis);
  }

  // Sort by score (best first)
  std::sort(results.begin(), results.end());

  return results;
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::pair<double, double>
ScoringSystem<NodeID, Pose2D, WeightType>::optimizeWeights(
    const std::vector<std::tuple<double, double, bool>> &trainingData) const {

  double bestLinkWeight  = 1.0;
  double bestGraphWeight = 1.0;
  int    bestScore       = 0;

  // Simple grid search
  for (double lw = 0.5; lw <= 3.0; lw += 0.1) {
    for (double gw = 0.5; gw <= 3.0; gw += 0.1) {
      int correctPredictions = 0;

      for (const auto &[linkDist, graphDist, optimal] : trainingData) {
        double score1 = calculateWeightedScore(linkDist, graphDist, lw, gw);
        double score2 =
            calculateWeightedScore(linkDist * 1.2, graphDist * 0.8, lw, gw);

        bool prediction = (score1 < score2);
        if (prediction == optimal) {
          correctPredictions++;
        }
      }

      if (correctPredictions > bestScore) {
        bestScore       = correctPredictions;
        bestLinkWeight  = lw;
        bestGraphWeight = gw;
      }
    }
  }

  return {bestLinkWeight, bestGraphWeight};
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::tuple<double, double, double>
ScoringSystem<NodeID, Pose2D, WeightType>::calculateMetrics(
    const std::vector<bool> &actualChoices,
    const std::vector<bool> &predictedChoices) const {

  if (actualChoices.size() != predictedChoices.size()) {
    throw std::invalid_argument("Choice vectors must have same size");
  }

  int truePositives = 0, falsePositives = 0, falseNegatives = 0,
      trueNegatives = 0;

  for (size_t i = 0; i < actualChoices.size(); ++i) {
    if (actualChoices[i] && predictedChoices[i])
      truePositives++;
    else if (!actualChoices[i] && predictedChoices[i])
      falsePositives++;
    else if (actualChoices[i] && !predictedChoices[i])
      falseNegatives++;
    else
      trueNegatives++;
  }

  double accuracy =
      static_cast<double>(truePositives + trueNegatives) / actualChoices.size();
  double precision = (truePositives + falsePositives > 0)
                         ? static_cast<double>(truePositives) /
                               (truePositives + falsePositives)
                         : 0.0;
  double recall    = (truePositives + falseNegatives > 0)
                         ? static_cast<double>(truePositives) /
                            (truePositives + falseNegatives)
                         : 0.0;

  return {accuracy, precision, recall};
}

// Explicit instantiations for the template methods
template double ScoringSystem<int, CPose2D, double>::calculateWeightedScore(
    double, double, double, double) const;
template double
ScoringSystem<unsigned long, CPose2D, double>::calculateWeightedScore(
    double, double, double, double) const;

template double
ScoringSystem<int, CPose2D, double>::calculateNormalizedWeightedScore(
    double, double, double, double, double, double) const;
template double
ScoringSystem<unsigned long, CPose2D, double>::calculateNormalizedWeightedScore(
    double, double, double, double, double, double) const;

template double
ScoringSystem<int, CPose2D, double>::calculateMultiCriteriaScore(
    double, double, int, double, const std::vector<double> &) const;
template double
ScoringSystem<unsigned long, CPose2D, double>::calculateMultiCriteriaScore(
    double, double, int, double, const std::vector<double> &) const;

template double ScoringSystem<int, CPose2D, double>::calculateAdaptiveScore(
    double, double, double, double) const;
template double
ScoringSystem<unsigned long, CPose2D, double>::calculateAdaptiveScore(
    double, double, double, double) const;

template std::vector<std::pair<double, std::string>>
ScoringSystem<int, CPose2D, double>::analyzeScoring(const std::vector<double> &,
                                                    const std::vector<double> &,
                                                    double, double) const;
template std::vector<std::pair<double, std::string>>
ScoringSystem<unsigned long, CPose2D, double>::analyzeScoring(
    const std::vector<double> &, const std::vector<double> &, double,
    double) const;

template std::pair<double, double>
ScoringSystem<int, CPose2D, double>::optimizeWeights(
    const std::vector<std::tuple<double, double, bool>> &) const;
template std::pair<double, double>
ScoringSystem<unsigned long, CPose2D, double>::optimizeWeights(
    const std::vector<std::tuple<double, double, bool>> &) const;

template std::tuple<double, double, double>
ScoringSystem<int, CPose2D, double>::calculateMetrics(
    const std::vector<bool> &, const std::vector<bool> &) const;
template std::tuple<double, double, double>
ScoringSystem<unsigned long, CPose2D, double>::calculateMetrics(
    const std::vector<bool> &, const std::vector<bool> &) const;

} // namespace algorithms
} // namespace mrpt_graphPose_pose