#include "vrobot_route_follow/core/rich_path_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace vrobot_route_follow {
namespace core {

// ========================================================================
// PRIMARY OPTIMIZATION INTERFACE
// ========================================================================

RichPathResult RichPathOptimizer::optimizePath(const RichPathResult& path,
                                               const OptimizationConfig& config,
                                               const std::optional<RobotConstraints>& constraints) {
    auto start_time = std::chrono::steady_clock::now();
    
    if (!path.success || path.poseSequence.empty()) {
        return path; // Cannot optimize invalid path
    }
    
    RichPathResult optimized_path = path;
    const RobotConstraints& robot_constraints = constraints ? *constraints : default_constraints_;
    
    // Apply optimizations based on configuration
    switch (config.primary_objective) {
        case OptimizationObjective::MINIMIZE_DISTANCE:
            optimized_path = optimizeForDistance(optimized_path, config.max_deviation_from_graph);
            break;
            
        case OptimizationObjective::MAXIMIZE_SMOOTHNESS:
            optimized_path = optimizeForSmoothness(optimized_path, config.smoothing_factor);
            break;
            
        case OptimizationObjective::MINIMIZE_TIME:
            optimized_path = optimizeVelocityProfile(optimized_path, robot_constraints);
            break;
            
        case OptimizationObjective::BALANCED:
            // Apply multiple optimizations with balanced weights
            if (config.enable_path_smoothing) {
                optimized_path = simplePathSmoothing(optimized_path, config);
            }
            if (config.enable_velocity_optimization) {
                optimized_path = velocityProfileOptimization(optimized_path, robot_constraints);
            }
            if (config.enable_turn_optimization) {
                optimizeTurnSequence(optimized_path, robot_constraints.min_turn_radius);
            }
            break;
            
        default:
            // Apply basic smoothing for other objectives
            optimized_path = simplePathSmoothing(optimized_path, config);
            break;
    }
    
    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    last_stats_.optimization_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    
    updateOptimizationStatistics(path, optimized_path, "comprehensive");
    
    return optimized_path;
}

RichPathResult RichPathOptimizer::quickOptimize(const RichPathResult& path,
                                                OptimizationObjective objective) {
    OptimizationConfig config;
    config.primary_objective = objective;
    return optimizePath(path, config);
}

// ========================================================================
// SPECIFIC OPTIMIZATION METHODS
// ========================================================================

RichPathResult RichPathOptimizer::optimizeForDistance(const RichPathResult& path,
                                                      double max_deviation) {
    if (path.poseSequence.size() < 3) {
        return path; // Cannot optimize very short paths
    }
    
    RichPathResult optimized_path = path;
    
    // Simple distance optimization: remove unnecessary waypoints
    std::vector<Eigen::Vector3d> optimized_poses;
    optimized_poses.push_back(path.poseSequence.front());
    
    for (size_t i = 1; i < path.poseSequence.size() - 1; ++i) {
        const auto& prev = optimized_poses.back();
        const auto& current = path.poseSequence[i];
        const auto& next = path.poseSequence[i + 1];
        
        // Check if current point can be skipped without exceeding deviation
        Eigen::Vector3d direct_vec = next - prev;
        Eigen::Vector3d current_vec = current - prev;
        
        // Project current onto direct line
        double t = current_vec.head<2>().dot(direct_vec.head<2>()) / 
                   direct_vec.head<2>().squaredNorm();
        t = std::clamp(t, 0.0, 1.0);
        
        Eigen::Vector3d projected = prev + t * direct_vec;
        double deviation = (current - projected).head<2>().norm();
        
        if (deviation > max_deviation) {
            optimized_poses.push_back(current);
        }
    }
    
    optimized_poses.push_back(path.poseSequence.back());
    optimized_path.poseSequence = optimized_poses;
    
    // Recalculate total distance
    optimized_path.totalDistance = 0.0;
    for (size_t i = 1; i < optimized_poses.size(); ++i) {
        optimized_path.totalDistance += 
            (optimized_poses[i] - optimized_poses[i-1]).head<2>().norm();
    }
    
    return optimized_path;
}

RichPathResult RichPathOptimizer::optimizeForSmoothness(const RichPathResult& path,
                                                        double smoothing_strength) {
    RichPathResult optimized_path = path;
    
    if (path.poseSequence.size() < 3) {
        return optimized_path;
    }
    
    // Apply smoothing to pose sequence
    size_t iterations = static_cast<size_t>(smoothing_strength * 10 + 1);
    optimized_path.poseSequence = smoothPoseSequence(
        path.poseSequence, smoothing_strength, iterations);
    
    // Recalculate total distance
    optimized_path.totalDistance = 0.0;
    for (size_t i = 1; i < optimized_path.poseSequence.size(); ++i) {
        optimized_path.totalDistance += 
            (optimized_path.poseSequence[i] - optimized_path.poseSequence[i-1]).head<2>().norm();
    }
    
    return optimized_path;
}

RichPathResult RichPathOptimizer::optimizeVelocityProfile(const RichPathResult& path,
                                                          const RobotConstraints& constraints) {
    RichPathResult optimized_path = path;
    optimizeVelocities(optimized_path, constraints);
    return optimized_path;
}

RichPathResult RichPathOptimizer::optimizeTurns(const RichPathResult& path,
                                                double min_turn_radius) {
    RichPathResult optimized_path = path;
    optimizeTurnSequence(optimized_path, min_turn_radius);
    return optimized_path;
}

// ========================================================================
// PATH ANALYSIS AND METRICS
// ========================================================================

std::unordered_map<std::string, double> RichPathOptimizer::analyzePath(const RichPathResult& path) {
    std::unordered_map<std::string, double> metrics;
    
    if (path.poseSequence.empty()) {
        return metrics;
    }
    
    metrics["total_distance"] = path.totalDistance;
    metrics["num_segments"] = static_cast<double>(path.poseSequence.size() - 1);
    metrics["distance_cost"] = calculateDistanceCost(path);
    metrics["smoothness_cost"] = calculateSmoothnessCost(path);
    metrics["turn_cost"] = calculateTurnCost(path);
    
    // Calculate average segment length
    if (path.poseSequence.size() > 1) {
        metrics["avg_segment_length"] = path.totalDistance / (path.poseSequence.size() - 1);
    }
    
    // Calculate path straightness (ratio of end-to-end distance to path distance)
    if (path.totalDistance > 0) {
        double end_to_end = (path.poseSequence.back() - path.poseSequence.front()).head<2>().norm();
        metrics["straightness"] = end_to_end / path.totalDistance;
    }
    
    return metrics;
}

std::unordered_map<std::string, double> RichPathOptimizer::comparePaths(const RichPathResult& path1,
                                                                        const RichPathResult& path2) {
    std::unordered_map<std::string, double> comparison;
    
    auto metrics1 = analyzePath(path1);
    auto metrics2 = analyzePath(path2);
    
    for (const auto& [key, value1] : metrics1) {
        auto it = metrics2.find(key);
        if (it != metrics2.end()) {
            double value2 = it->second;
            double improvement = (value1 - value2) / (value1 + 1e-9) * 100.0; // Percentage improvement
            comparison[key + "_improvement_percent"] = improvement;
            comparison[key + "_ratio"] = value2 / (value1 + 1e-9);
        }
    }
    
    return comparison;
}

double RichPathOptimizer::calculatePathCost(const RichPathResult& path,
                                           OptimizationObjective objective,
                                           const OptimizationConfig& config) {
    switch (objective) {
        case OptimizationObjective::MINIMIZE_DISTANCE:
            return calculateDistanceCost(path);
            
        case OptimizationObjective::MAXIMIZE_SMOOTHNESS:
            return -calculateSmoothnessCost(path); // Negative because we maximize smoothness
            
        case OptimizationObjective::MINIMIZE_TURNS:
            return calculateTurnCost(path);
            
        case OptimizationObjective::BALANCED:
            return config.distance_weight * calculateDistanceCost(path) +
                   config.smoothness_weight * calculateSmoothnessCost(path) +
                   config.turn_weight * calculateTurnCost(path) +
                   config.velocity_weight * calculateVelocityCost(path);
            
        default:
            return calculateDistanceCost(path);
    }
}

// ========================================================================
// STATISTICS AND DIAGNOSTICS
// ========================================================================

std::unordered_map<std::string, double> RichPathOptimizer::getAggregatedStats() const {
    std::unordered_map<std::string, double> stats;
    
    stats["total_optimizations"] = static_cast<double>(total_optimizations_);
    stats["total_optimization_time_ms"] = static_cast<double>(total_optimization_time_.count());
    
    if (total_optimizations_ > 0) {
        stats["avg_optimization_time_ms"] = static_cast<double>(total_optimization_time_.count()) / 
                                           total_optimizations_;
    }
    
    // Method usage statistics
    for (const auto& [method, count] : method_usage_count_) {
        stats["usage_" + method] = static_cast<double>(count);
    }
    
    return stats;
}

void RichPathOptimizer::clearStatistics() {
    last_stats_ = OptimizationStatistics{};
    total_optimizations_ = 0;
    total_optimization_time_ = std::chrono::milliseconds{0};
    method_usage_count_.clear();
    objective_usage_count_.clear();
}

// ========================================================================
// CONFIGURATION MANAGEMENT
// ========================================================================

void RichPathOptimizer::setDefaultConfig(const OptimizationConfig& config) {
    default_config_ = config;
}

void RichPathOptimizer::setDefaultRobotConstraints(const RobotConstraints& constraints) {
    default_constraints_ = constraints;
}

// ========================================================================
// INTERNAL OPTIMIZATION METHODS
// ========================================================================

RichPathResult RichPathOptimizer::simplePathSmoothing(const RichPathResult& path,
                                                      const OptimizationConfig& config) {
    RichPathResult smoothed_path = path;
    
    if (config.enable_path_smoothing && path.poseSequence.size() > 2) {
        smoothed_path.poseSequence = smoothPoseSequence(
            path.poseSequence, 
            config.smoothing_factor, 
            config.smoothing_iterations
        );
        
        // Recalculate distance
        smoothed_path.totalDistance = 0.0;
        for (size_t i = 1; i < smoothed_path.poseSequence.size(); ++i) {
            smoothed_path.totalDistance += 
                (smoothed_path.poseSequence[i] - smoothed_path.poseSequence[i-1]).head<2>().norm();
        }
    }
    
    return smoothed_path;
}

std::vector<Eigen::Vector3d> RichPathOptimizer::smoothPoseSequence(const std::vector<Eigen::Vector3d>& poses,
                                                                   double smoothing_factor,
                                                                   size_t iterations) {
    if (poses.size() < 3) {
        return poses;
    }
    
    std::vector<Eigen::Vector3d> smoothed = poses;
    
    for (size_t iter = 0; iter < iterations; ++iter) {
        std::vector<Eigen::Vector3d> temp = smoothed;
        
        // Apply smoothing to intermediate points (preserve start and end)
        for (size_t i = 1; i < smoothed.size() - 1; ++i) {
            Eigen::Vector3d avg = (smoothed[i-1] + smoothed[i+1]) * 0.5;
            temp[i] = smoothed[i] + smoothing_factor * (avg - smoothed[i]);
        }
        
        smoothed = temp;
    }
    
    return smoothed;
}

RichPathResult RichPathOptimizer::velocityProfileOptimization(const RichPathResult& path,
                                                           const RobotConstraints& constraints) {
    RichPathResult optimized_path = path;
    
    if (path.poseSequence.size() < 2) {
        return optimized_path;
    }
    
    // Advanced velocity profile optimization
    // Calculate optimal velocities considering acceleration limits and curvature
    for (size_t i = 0; i < optimized_path.linkSequence.size(); ++i) {
        auto& link = optimized_path.linkSequence[i];
        
        // Apply basic velocity constraints
        if (link.max_velocity > constraints.max_linear_velocity) {
            link.max_velocity = constraints.max_linear_velocity;
        }
        
        // Apply curvature-based velocity reduction for turns
        if (i + 2 < path.poseSequence.size()) {
            double curvature = calculateCurvature(
                path.poseSequence[i],
                path.poseSequence[i + 1],
                path.poseSequence[i + 2]
            );
            
            // Reduce velocity in high curvature areas
            if (curvature > 0.1) { // High curvature threshold
                double curvature_factor = 1.0 / (1.0 + curvature * 5.0);
                link.max_velocity *= curvature_factor;
            }
        }
        
        // Ensure minimum velocity
        if (link.max_velocity < constraints.comfortable_linear_velocity) {
            link.max_velocity = constraints.comfortable_linear_velocity;
        }
    }
    
    return optimized_path;
}

void RichPathOptimizer::optimizeVelocities(RichPathResult& path,
                                          const RobotConstraints& constraints) {
    // Simple velocity profile optimization
    // This is a placeholder implementation
    // TODO: Implement sophisticated velocity profile optimization
    
    if (path.poseSequence.size() < 2) {
        return;
    }
    
    // For now, just ensure velocities don't exceed constraints
    for (auto& link : path.linkSequence) {
        if (link.max_velocity > constraints.max_linear_velocity) {
            link.max_velocity = constraints.max_linear_velocity;
        }
    }
}

void RichPathOptimizer::optimizeTurnSequence(RichPathResult& path,
                                            double min_turn_radius) {
    // Simple turn optimization
    // This is a placeholder implementation
    // TODO: Implement sophisticated turn optimization
    
    if (path.poseSequence.size() < 3) {
        return;
    }
    
    // Check and adjust sharp turns
    for (size_t i = 1; i < path.poseSequence.size() - 1; ++i) {
        double curvature = calculateCurvature(
            path.poseSequence[i-1],
            path.poseSequence[i],
            path.poseSequence[i+1]
        );
        
        if (curvature > 1.0 / min_turn_radius) {
            // Turn is too sharp, could apply some correction here
            // For now, just log it
        }
    }
}

// ========================================================================
// COST FUNCTIONS
// ========================================================================

double RichPathOptimizer::calculateDistanceCost(const RichPathResult& path) {
    return path.totalDistance;
}

double RichPathOptimizer::calculateSmoothnessCost(const RichPathResult& path) {
    if (path.poseSequence.size() < 3) {
        return 0.0;
    }
    
    double total_curvature = 0.0;
    for (size_t i = 1; i < path.poseSequence.size() - 1; ++i) {
        total_curvature += calculateCurvature(
            path.poseSequence[i-1],
            path.poseSequence[i],
            path.poseSequence[i+1]
        );
    }
    
    return total_curvature;
}

double RichPathOptimizer::calculateVelocityCost(const RichPathResult& path) {
    // Simple velocity cost based on maximum velocities
    double total_time = 0.0;
    
    for (size_t i = 1; i < path.poseSequence.size(); ++i) {
        double segment_distance = (path.poseSequence[i] - path.poseSequence[i-1]).head<2>().norm();
        double velocity = 1.0; // Default velocity
        
        // Use link velocity if available
        if (i-1 < path.linkSequence.size() && path.linkSequence[i-1].max_velocity > 0) {
            velocity = path.linkSequence[i-1].max_velocity;
        }
        
        total_time += segment_distance / velocity;
    }
    
    return total_time;
}

double RichPathOptimizer::calculateTurnCost(const RichPathResult& path) {
    if (path.poseSequence.size() < 3) {
        return 0.0;
    }
    
    double total_turn_cost = 0.0;
    for (size_t i = 1; i < path.poseSequence.size() - 1; ++i) {
        double turn_angle = std::abs(calculateTurnAngle(
            path.poseSequence[i-1],
            path.poseSequence[i],
            path.poseSequence[i+1]
        ));
        
        total_turn_cost += turn_angle;
    }
    
    return total_turn_cost;
}

double RichPathOptimizer::calculateSafetyCost(const RichPathResult& path) {
    // Placeholder for safety cost calculation
    // Could include factors like proximity to obstacles, path clearance, etc.
    return 0.0;
}

// ========================================================================
// UTILITY METHODS
// ========================================================================

double RichPathOptimizer::calculateCurvature(const Eigen::Vector3d& p1,
                                             const Eigen::Vector3d& p2,
                                             const Eigen::Vector3d& p3) {
    Eigen::Vector2d v1 = p2.head<2>() - p1.head<2>();
    Eigen::Vector2d v2 = p3.head<2>() - p2.head<2>();
    
    double cross_product = v1.x() * v2.y() - v1.y() * v2.x();
    double len1 = v1.norm();
    double len2 = v2.norm();
    
    if (len1 < 1e-9 || len2 < 1e-9) {
        return 0.0;
    }
    
    return std::abs(cross_product) / (len1 * len2);
}

double RichPathOptimizer::calculateTurnAngle(const Eigen::Vector3d& p1,
                                             const Eigen::Vector3d& p2,
                                             const Eigen::Vector3d& p3) {
    Eigen::Vector2d v1 = (p2 - p1).head<2>().normalized();
    Eigen::Vector2d v2 = (p3 - p2).head<2>().normalized();
    
    double dot_product = v1.dot(v2);
    dot_product = std::clamp(dot_product, -1.0, 1.0);
    
    return std::acos(dot_product);
}

void RichPathOptimizer::updateOptimizationStatistics(const RichPathResult& original_path,
                                                     const RichPathResult& optimized_path,
                                                     const std::string& method) {
    last_stats_.optimization_method = method;
    
    // Calculate improvements
    last_stats_.distance_before = original_path.totalDistance;
    last_stats_.distance_after = optimized_path.totalDistance;
    
    last_stats_.smoothness_before = calculateSmoothnessCost(original_path);
    last_stats_.smoothness_after = calculateSmoothnessCost(optimized_path);
    
    // Calculate improvement percentage
    if (last_stats_.distance_before > 0) {
        last_stats_.improvement_percentage = 
            (last_stats_.distance_before - last_stats_.distance_after) / 
            last_stats_.distance_before * 100.0;
    }
    
    // Update global statistics
    ++total_optimizations_;
    total_optimization_time_ += last_stats_.optimization_time;
    ++method_usage_count_[method];
}

} // namespace core
} // namespace vrobot_route_follow