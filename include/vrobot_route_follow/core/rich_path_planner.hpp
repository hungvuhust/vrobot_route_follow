#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

#include "vrobot_route_follow/algorithms/algorithm_interface.hpp"
#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/rich_path_result.hpp"
#include "vrobot_route_follow/core/rich_database_loader.hpp"

using vrobot_route_follow::data_structures::NodeInfo;
using vrobot_route_follow::data_structures::LinkInfo;
using vrobot_route_follow::data_structures::RichPathResult;

namespace vrobot_route_follow {
namespace core {

/**
 * @brief Planning request structure
 */
struct PlanningRequest {
    // Start configuration
    Eigen::Vector3d start_pose;        // (x, y, theta)
    std::optional<int32_t> start_node_id; // If starting from a specific node
    
    // Goal configuration
    std::optional<Eigen::Vector3d> goal_pose;    // (x, y, theta) 
    std::optional<int32_t> goal_node_id;         // If going to specific node
    
    // Planning parameters
    algorithms::AlgorithmConfig algorithm_config;
    
    // Request metadata
    std::string request_id;
    std::chrono::steady_clock::time_point timestamp;
    int32_t priority = 0; // Higher values = higher priority
    
    // Validation
    bool isValid() const {
        return (goal_pose || goal_node_id) && 
               !request_id.empty();
    }
};

/**
 * @brief Planning statistics and diagnostics
 */
struct PlanningStatistics {
    // Timing information
    std::chrono::milliseconds total_time{0};
    std::chrono::milliseconds algorithm_time{0};
    std::chrono::milliseconds preprocessing_time{0};
    std::chrono::milliseconds postprocessing_time{0};
    
    // Search space information
    size_t nodes_explored = 0;
    size_t links_evaluated = 0;
    size_t alternative_paths_considered = 0;
    
    // Path quality metrics
    double path_length = 0.0;
    double path_smoothness = 0.0;
    size_t path_segments = 0;
    size_t direction_changes = 0;
    
    // Algorithm information
    std::string algorithm_used;
    bool fallback_used = false;
    std::vector<std::string> warnings;
    
    // Resource usage
    size_t memory_peak_kb = 0;
    double cpu_usage_percent = 0.0;
};

/**
 * @brief Planning context for advanced scenarios
 */
struct PlanningContext {
    // Dynamic obstacles (if available)
    std::vector<std::pair<Eigen::Vector2d, double>> dynamic_obstacles; // center, radius
    
    // Time constraints
    std::optional<std::chrono::milliseconds> max_planning_time;
    std::optional<std::chrono::steady_clock::time_point> deadline;
    
    // Robot constraints
    std::optional<double> robot_width;
    std::optional<double> robot_length;
    std::optional<double> max_turn_radius;
    
    // Map region constraints
    std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> bounding_box; // min, max
    std::vector<int32_t> forbidden_nodes;
    std::vector<int32_t> forbidden_links;
    
    // Multi-goal planning
    std::vector<int32_t> waypoint_nodes; // Must visit these nodes in order
    bool optimize_waypoint_order = false;
    
    // Replanning context
    std::optional<RichPathResult> previous_path;
    bool prefer_path_continuity = false;
    double path_deviation_penalty = 0.0;
};

/**
 * @brief Core path planning orchestrator
 * 
 * This class coordinates all aspects of path planning:
 * - Request validation and preprocessing
 * - Algorithm selection and execution
 * - Result validation and postprocessing
 * - Performance monitoring and diagnostics
 * - Fallback handling and error recovery
 */
class RichPathPlanner {
public:
    // Constructor and destructor
    explicit RichPathPlanner(std::shared_ptr<RichDatabaseLoader> data_loader);
    ~RichPathPlanner() = default;

    // Copy/move semantics
    RichPathPlanner(const RichPathPlanner&) = delete;
    RichPathPlanner& operator=(const RichPathPlanner&) = delete;
    RichPathPlanner(RichPathPlanner&&) = default;
    RichPathPlanner& operator=(RichPathPlanner&&) = default;

    // ========================================================================
    // PRIMARY PLANNING INTERFACE
    // ========================================================================

    /**
     * @brief Plan path with comprehensive configuration
     * @param request Planning request with all parameters
     * @param context Optional advanced planning context
     * @return Rich path result with full traceability
     */
    RichPathResult planPath(const PlanningRequest& request, 
                           const PlanningContext& context = PlanningContext{});

    /**
     * @brief Simple pose-to-pose planning (backward compatibility)
     * @param start_pose Starting pose (x, y, theta)
     * @param goal_pose Goal pose (x, y, theta)
     * @param config Algorithm configuration
     * @return Rich path result
     */
    RichPathResult planPoseToPose(const Eigen::Vector3d& start_pose,
                                 const Eigen::Vector3d& goal_pose,
                                 const algorithms::AlgorithmConfig& config = algorithms::AlgorithmConfig{});

    /**
     * @brief Plan from pose to specific node
     * @param start_pose Starting pose (x, y, theta)
     * @param goal_node_id Target node ID
     * @param config Algorithm configuration
     * @return Rich path result
     */
    RichPathResult planPoseToNode(const Eigen::Vector3d& start_pose,
                                 int32_t goal_node_id,
                                 const algorithms::AlgorithmConfig& config = algorithms::AlgorithmConfig{});

    /**
     * @brief Plan between specific nodes
     * @param start_node_id Starting node ID
     * @param goal_node_id Target node ID  
     * @param config Algorithm configuration
     * @return Rich path result
     */
    RichPathResult planNodeToNode(int32_t start_node_id,
                                 int32_t goal_node_id,
                                 const algorithms::AlgorithmConfig& config = algorithms::AlgorithmConfig{});

    // ========================================================================
    // ADVANCED PLANNING FEATURES
    // ========================================================================

    /**
     * @brief Multi-waypoint planning
     * @param start_pose Starting pose
     * @param waypoint_nodes List of waypoint node IDs to visit in order
     * @param goal_pose Final goal pose (optional)
     * @param config Algorithm configuration
     * @return Rich path result visiting all waypoints
     */
    RichPathResult planMultiWaypoint(const Eigen::Vector3d& start_pose,
                                    const std::vector<int32_t>& waypoint_nodes,
                                    const std::optional<Eigen::Vector3d>& goal_pose = std::nullopt,
                                    const algorithms::AlgorithmConfig& config = algorithms::AlgorithmConfig{});

    /**
     * @brief Replan from current position, considering previous path
     * @param current_pose Current robot pose
     * @param previous_path Previous planned path
     * @param goal_node_id Target node ID
     * @param config Algorithm configuration
     * @return Updated rich path result
     */
    RichPathResult replanFromCurrent(const Eigen::Vector3d& current_pose,
                                    const RichPathResult& previous_path,
                                    int32_t goal_node_id,
                                    const algorithms::AlgorithmConfig& config = algorithms::AlgorithmConfig{});

    /**
     * @brief Plan with time constraints
     * @param request Planning request
     * @param max_time Maximum planning time allowed
     * @return Best path found within time limit
     */
    RichPathResult planWithTimeLimit(const PlanningRequest& request,
                                    std::chrono::milliseconds max_time);

    // ========================================================================
    // ALGORITHM MANAGEMENT
    // ========================================================================

    /**
     * @brief Get available algorithms
     * @return List of algorithm names
     */
    std::vector<std::string> getAvailableAlgorithms() const;

    /**
     * @brief Set default algorithm for planning
     * @param algorithm_name Name of the algorithm to use as default
     */
    void setDefaultAlgorithm(const std::string& algorithm_name);

    /**
     * @brief Register custom algorithm
     * @param algorithm Shared pointer to algorithm implementation
     */
    void registerAlgorithm(std::shared_ptr<algorithms::PathPlanningAlgorithm> algorithm);

    // ========================================================================
    // STATISTICS AND DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get statistics from last planning operation
     * @return Detailed planning statistics
     */
    const PlanningStatistics& getLastPlanningStats() const { return last_stats_; }

    /**
     * @brief Get aggregated statistics over all planning operations
     * @return Aggregated statistics
     */
    std::unordered_map<std::string, double> getAggregatedStats() const;

    /**
     * @brief Clear all statistics
     */
    void clearStatistics();

    /**
     * @brief Enable/disable detailed performance profiling
     * @param enable True to enable profiling
     */
    void setProfilingEnabled(bool enable) { profiling_enabled_ = enable; }

    // ========================================================================
    // CONFIGURATION MANAGEMENT
    // ========================================================================

    /**
     * @brief Set global planning configuration
     * @param config Default configuration to use
     */
    void setDefaultConfig(const algorithms::AlgorithmConfig& config);

    /**
     * @brief Get current default configuration
     * @return Current default configuration
     */
    const algorithms::AlgorithmConfig& getDefaultConfig() const { return default_config_; }

    /**
     * @brief Set fallback algorithms in priority order
     * @param fallback_algorithms List of algorithm names to try if primary fails
     */
    void setFallbackAlgorithms(const std::vector<std::string>& fallback_algorithms);

    // ========================================================================
    // VALIDATION AND HEALTH CHECKS
    // ========================================================================

    /**
     * @brief Validate that planner is ready for operations
     * @return Validation results
     */
    std::vector<std::string> validatePlannerHealth() const;

    /**
     * @brief Test planning with known good configuration
     * @return True if test planning succeeds
     */
    bool runSelfTest();

private:
    // ========================================================================
    // INTERNAL IMPLEMENTATION
    // ========================================================================

    // Core planning workflow
    RichPathResult executePlanningWorkflow(const PlanningRequest& request,
                                          const PlanningContext& context);
    
    // Request preprocessing
    PlanningRequest preprocessRequest(const PlanningRequest& request,
                                     const PlanningContext& context);
    std::optional<int32_t> findClosestNode(const Eigen::Vector3d& pose);
    bool validatePlanningRequest(const PlanningRequest& request);
    
    // Algorithm execution
    RichPathResult executeAlgorithm(const std::string& algorithm_name,
                                   const PlanningRequest& request,
                                   const PlanningContext& context);
    RichPathResult executeFallbackAlgorithms(const PlanningRequest& request,
                                            const PlanningContext& context);
    
    // Result postprocessing
    RichPathResult postprocessResult(RichPathResult result,
                                    const PlanningRequest& request,
                                    const PlanningContext& context);
    void validateResult(const RichPathResult& result);
    
    // Performance monitoring
    void startProfiling();
    void stopProfiling();
    void updateStatistics(const RichPathResult& result,
                         const std::string& algorithm_used);
    
    // Utility methods
    double calculatePathSmoothness(const RichPathResult& result);
    size_t countDirectionChanges(const RichPathResult& result);
    std::string generateRequestId();

    // Member variables
    std::shared_ptr<RichDatabaseLoader> data_loader_;
    std::shared_ptr<algorithms::AlgorithmManager> algorithm_manager_;
    
    // Configuration
    algorithms::AlgorithmConfig default_config_;
    std::vector<std::string> fallback_algorithms_;
    std::string default_algorithm_ = "dijkstra";
    
    // Performance monitoring
    bool profiling_enabled_ = false;
    PlanningStatistics last_stats_;
    std::unordered_map<std::string, size_t> algorithm_usage_count_;
    std::unordered_map<std::string, std::chrono::milliseconds> algorithm_total_time_;
    std::chrono::steady_clock::time_point profiling_start_;
    
    // Request tracking
    std::unordered_map<std::string, PlanningRequest> active_requests_;
    size_t total_requests_processed_ = 0;
    size_t successful_requests_ = 0;
    
    // Health monitoring
    std::chrono::steady_clock::time_point last_successful_plan_;
    size_t consecutive_failures_ = 0;
};

} // namespace core
} // namespace vrobot_route_follow