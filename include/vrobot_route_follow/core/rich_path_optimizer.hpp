#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>

#include <Eigen/Dense>

#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"

using vrobot_route_follow::data_structures::NodeInfo;
using vrobot_route_follow::data_structures::LinkInfo;
using vrobot_route_follow::data_structures::RichPathResult;

namespace vrobot_route_follow {
namespace core {

/**
 * @brief Optimization objective types
 */
enum class OptimizationObjective {
    MINIMIZE_DISTANCE,      // Shortest path
    MINIMIZE_TIME,          // Fastest path considering velocities
    MAXIMIZE_SMOOTHNESS,    // Smoothest path with minimal curvature
    MINIMIZE_ENERGY,        // Energy-efficient path
    MINIMIZE_TURNS,         // Fewest direction changes
    BALANCED,               // Balanced optimization of multiple objectives
    CUSTOM                  // User-defined objective function
};

/**
 * @brief Optimization configuration
 */
struct OptimizationConfig {
    // Primary objective
    OptimizationObjective primary_objective = OptimizationObjective::BALANCED;
    
    // Objective weights (for balanced optimization)
    double distance_weight = 1.0;
    double smoothness_weight = 0.5;
    double velocity_weight = 0.3;
    double turn_weight = 0.2;
    double safety_weight = 1.0;
    
    // Smoothing parameters
    bool enable_path_smoothing = true;
    double smoothing_factor = 0.1;           // Strength of smoothing
    size_t smoothing_iterations = 3;         // Number of smoothing passes
    double max_deviation_from_graph = 0.5;   // Maximum allowed deviation from original path
    
    // Velocity optimization
    bool enable_velocity_optimization = true;
    double default_velocity = 1.0;           // m/s
    double max_acceleration = 2.0;           // m/s²
    double max_deceleration = 3.0;           // m/s²
    double comfort_acceleration = 1.0;       // m/s² for comfort
    
    // Turn optimization
    bool enable_turn_optimization = true;
    double max_turn_rate = 0.5;              // rad/s
    double turn_anticipation_distance = 1.0; // Look-ahead distance for turns
    
    // Advanced optimization
    bool enable_obstacle_avoidance = false;
    bool enable_dynamic_optimization = false;
    bool preserve_waypoints = true;          // Keep specified waypoints unchanged
    
    // Performance constraints
    std::chrono::milliseconds max_optimization_time{1000};
    size_t max_iterations = 100;
    double convergence_threshold = 0.001;
    
    // Custom objective function (for CUSTOM objective)
    std::function<double(const RichPathResult&)> custom_objective = nullptr;
};

/**
 * @brief Optimization statistics
 */
struct OptimizationStatistics {
    // Performance metrics
    std::chrono::milliseconds optimization_time{0};
    size_t iterations_performed = 0;
    bool converged = false;
    
    // Quality improvements
    double initial_cost = 0.0;
    double final_cost = 0.0;
    double improvement_percentage = 0.0;
    
    // Path characteristics (before -> after)
    double distance_before = 0.0, distance_after = 0.0;
    double smoothness_before = 0.0, smoothness_after = 0.0;
    size_t turns_before = 0, turns_after = 0;
    double max_velocity_before = 0.0, max_velocity_after = 0.0;
    
    // Optimization details
    std::string optimization_method;
    std::vector<std::string> optimizations_applied;
    std::vector<std::string> warnings;
};

/**
 * @brief Robot constraints for optimization
 */
struct RobotConstraints {
    // Physical dimensions
    double width = 0.6;      // m
    double length = 1.0;     // m
    double height = 0.5;     // m
    
    // Kinematic constraints
    double max_linear_velocity = 2.0;       // m/s
    double max_angular_velocity = 1.0;      // rad/s
    double max_linear_acceleration = 2.0;   // m/s²
    double max_angular_acceleration = 2.0;  // rad/s²
    
    // Comfort constraints
    double comfortable_linear_velocity = 1.0;    // m/s
    double comfortable_angular_velocity = 0.5;   // rad/s
    double comfortable_acceleration = 1.0;       // m/s²
    
    // Safety margins
    double safety_margin = 0.2;             // m (additional clearance)
    double emergency_brake_distance = 1.0;  // m
    
    // Turn constraints
    double min_turn_radius = 0.5;           // m
    double preferred_turn_radius = 1.0;     // m
};

/**
 * @brief Advanced path optimizer for rich path results
 * 
 * This class provides comprehensive path optimization capabilities:
 * - Multiple optimization objectives (distance, time, smoothness, energy)
 * - Path smoothing with graph constraints
 * - Velocity profile optimization
 * - Turn optimization and cornering
 * - Robot constraint satisfaction
 * - Multi-objective optimization with configurable weights
 */
class RichPathOptimizer {
public:
    // Constructor and destructor
    RichPathOptimizer() = default;
    ~RichPathOptimizer() = default;

    // Copy/move semantics
    RichPathOptimizer(const RichPathOptimizer&) = default;
    RichPathOptimizer& operator=(const RichPathOptimizer&) = default;
    RichPathOptimizer(RichPathOptimizer&&) = default;
    RichPathOptimizer& operator=(RichPathOptimizer&&) = default;

    // ========================================================================
    // PRIMARY OPTIMIZATION INTERFACE
    // ========================================================================

    /**
     * @brief Optimize path with comprehensive configuration
     * @param path Input path to optimize
     * @param config Optimization configuration
     * @param constraints Robot constraints (optional)
     * @return Optimized path result
     */
    RichPathResult optimizePath(const RichPathResult& path,
                               const OptimizationConfig& config = OptimizationConfig{},
                               const std::optional<RobotConstraints>& constraints = std::nullopt);

    /**
     * @brief Quick optimization with default settings
     * @param path Input path to optimize
     * @param objective Primary optimization objective
     * @return Optimized path result
     */
    RichPathResult quickOptimize(const RichPathResult& path,
                                OptimizationObjective objective = OptimizationObjective::BALANCED);

    // ========================================================================
    // SPECIFIC OPTIMIZATION METHODS
    // ========================================================================

    /**
     * @brief Optimize for minimum distance
     * @param path Input path
     * @param max_deviation Maximum allowed deviation from original path
     * @return Distance-optimized path
     */
    RichPathResult optimizeForDistance(const RichPathResult& path,
                                      double max_deviation = 0.5);

    /**
     * @brief Optimize for maximum smoothness
     * @param path Input path
     * @param smoothing_strength Strength of smoothing operation [0, 1]
     * @return Smoothness-optimized path
     */
    RichPathResult optimizeForSmoothness(const RichPathResult& path,
                                        double smoothing_strength = 0.5);

    /**
     * @brief Optimize velocity profile
     * @param path Input path
     * @param constraints Robot kinematic constraints
     * @return Velocity-optimized path
     */
    RichPathResult optimizeVelocityProfile(const RichPathResult& path,
                                          const RobotConstraints& constraints);

    /**
     * @brief Optimize turns and cornering
     * @param path Input path
     * @param min_turn_radius Minimum turning radius
     * @return Turn-optimized path
     */
    RichPathResult optimizeTurns(const RichPathResult& path,
                                double min_turn_radius = 0.5);

    // ========================================================================
    // ADVANCED OPTIMIZATION FEATURES
    // ========================================================================

    /**
     * @brief Multi-objective optimization
     * @param path Input path
     * @param objectives Vector of objectives with weights
     * @return Multi-objective optimized path
     */
    RichPathResult multiObjectiveOptimization(const RichPathResult& path,
                                             const std::vector<std::pair<OptimizationObjective, double>>& objectives);

    /**
     * @brief Iterative optimization with convergence
     * @param path Input path
     * @param config Optimization configuration
     * @param max_iterations Maximum number of iterations
     * @return Iteratively optimized path
     */
    RichPathResult iterativeOptimization(const RichPathResult& path,
                                        const OptimizationConfig& config,
                                        size_t max_iterations = 10);

    /**
     * @brief Local optimization around specific segments
     * @param path Input path
     * @param segment_indices Indices of segments to optimize locally
     * @param config Optimization configuration
     * @return Locally optimized path
     */
    RichPathResult localOptimization(const RichPathResult& path,
                                    const std::vector<size_t>& segment_indices,
                                    const OptimizationConfig& config);

    // ========================================================================
    // PATH ANALYSIS AND METRICS
    // ========================================================================

    /**
     * @brief Analyze path quality
     * @param path Path to analyze
     * @return Quality metrics
     */
    std::unordered_map<std::string, double> analyzePath(const RichPathResult& path);

    /**
     * @brief Compare two paths
     * @param path1 First path
     * @param path2 Second path  
     * @return Comparison metrics
     */
    std::unordered_map<std::string, double> comparePaths(const RichPathResult& path1,
                                                         const RichPathResult& path2);

    /**
     * @brief Calculate path cost for given objective
     * @param path Path to evaluate
     * @param objective Optimization objective
     * @param weights Objective weights (for balanced optimization)
     * @return Path cost value
     */
    double calculatePathCost(const RichPathResult& path,
                            OptimizationObjective objective,
                            const OptimizationConfig& config = OptimizationConfig{});

    // ========================================================================
    // STATISTICS AND DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get statistics from last optimization
     * @return Detailed optimization statistics
     */
    const OptimizationStatistics& getLastOptimizationStats() const { return last_stats_; }

    /**
     * @brief Get aggregated optimization statistics
     * @return Aggregated statistics over all optimizations
     */
    std::unordered_map<std::string, double> getAggregatedStats() const;

    /**
     * @brief Clear all statistics
     */
    void clearStatistics();

    // ========================================================================
    // CONFIGURATION MANAGEMENT
    // ========================================================================

    /**
     * @brief Set default optimization configuration
     * @param config Default configuration to use
     */
    void setDefaultConfig(const OptimizationConfig& config);

    /**
     * @brief Get current default configuration
     * @return Current default configuration
     */
    const OptimizationConfig& getDefaultConfig() const { return default_config_; }

    /**
     * @brief Set default robot constraints
     * @param constraints Default robot constraints
     */
    void setDefaultRobotConstraints(const RobotConstraints& constraints);

    /**
     * @brief Get current default robot constraints
     * @return Current default robot constraints
     */
    const RobotConstraints& getDefaultRobotConstraints() const { return default_constraints_; }

private:
    // ========================================================================
    // INTERNAL OPTIMIZATION METHODS
    // ========================================================================

    // Core optimization algorithms
    RichPathResult gradientDescentOptimization(const RichPathResult& path,
                                              const OptimizationConfig& config,
                                              const RobotConstraints& constraints);
    
    RichPathResult simplePathSmoothing(const RichPathResult& path,
                                      const OptimizationConfig& config);
    
    RichPathResult velocityProfileOptimization(const RichPathResult& path,
                                              const RobotConstraints& constraints);
    
    // Specific optimization components
    std::vector<Eigen::Vector3d> smoothPoseSequence(const std::vector<Eigen::Vector3d>& poses,
                                                    double smoothing_factor,
                                                    size_t iterations);
    
    void optimizeVelocities(RichPathResult& path,
                           const RobotConstraints& constraints);
    
    void optimizeTurnSequence(RichPathResult& path,
                             double min_turn_radius);
    
    // Cost functions
    double calculateDistanceCost(const RichPathResult& path);
    double calculateSmoothnessCost(const RichPathResult& path);
    double calculateVelocityCost(const RichPathResult& path);
    double calculateTurnCost(const RichPathResult& path);
    double calculateSafetyCost(const RichPathResult& path);
    
    // Constraint checking
    bool satisfiesConstraints(const RichPathResult& path,
                             const RobotConstraints& constraints);
    
    std::vector<std::string> validateConstraints(const RichPathResult& path,
                                                const RobotConstraints& constraints);
    
    // Utility methods
    double calculateCurvature(const Eigen::Vector3d& p1,
                             const Eigen::Vector3d& p2,
                             const Eigen::Vector3d& p3);
    
    double calculateTurnAngle(const Eigen::Vector3d& p1,
                             const Eigen::Vector3d& p2,
                             const Eigen::Vector3d& p3);
    
    void updateOptimizationStatistics(const RichPathResult& original_path,
                                     const RichPathResult& optimized_path,
                                     const std::string& method);

    // Member variables
    OptimizationConfig default_config_;
    RobotConstraints default_constraints_;
    
    // Statistics tracking
    OptimizationStatistics last_stats_;
    size_t total_optimizations_ = 0;
    std::chrono::milliseconds total_optimization_time_{0};
    std::unordered_map<std::string, size_t> method_usage_count_;
    std::unordered_map<OptimizationObjective, size_t> objective_usage_count_;
};

} // namespace core
} // namespace vrobot_route_follow