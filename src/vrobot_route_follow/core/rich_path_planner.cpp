#include "vrobot_route_follow/core/rich_path_planner.hpp"
#include "vrobot_route_follow/algorithms/algorithm_interface.hpp"
#include "vrobot_route_follow/core/rich_geometric_utils.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <sstream>
#include <thread>

namespace vrobot_route_follow {
namespace core {

RichPathPlanner::RichPathPlanner(std::shared_ptr<RichDatabaseLoader> data_loader)
    : data_loader_(std::move(data_loader)) {
    if (!data_loader_) {
        throw std::invalid_argument("Data loader cannot be null");
    }
    
    // Note: Algorithm manager will be initialized when needed
    // algorithm_manager_ = std::make_shared<algorithms::AlgorithmManager>(graph_data);
    
    // Set default fallback algorithms
    fallback_algorithms_ = {"dijkstra", "a_star", "direct_path"};
    
    // Initialize timing
    last_successful_plan_ = std::chrono::steady_clock::now();
}

// ========================================================================
// PRIMARY PLANNING INTERFACE
// ========================================================================

RichPathResult RichPathPlanner::planPath(const PlanningRequest& request, 
                                         const PlanningContext& context) {
    auto start_time = std::chrono::steady_clock::now();
    
    try {
        // Start profiling if enabled
        if (profiling_enabled_) {
            startProfiling();
        }
        
        // Validate request
        if (!validatePlanningRequest(request)) {
            RichPathResult error_result;
            error_result.success = false;
            error_result.algorithmUsed = "validation_failed";
            return error_result;
        }
        
        // Execute planning workflow
        auto result = executePlanningWorkflow(request, context);
        
        // Update statistics
        updateStatistics(result, result.algorithmUsed);
        
        // Stop profiling
        if (profiling_enabled_) {
            stopProfiling();
        }
        
        // Track successful planning
        if (result.success) {
            last_successful_plan_ = std::chrono::steady_clock::now();
            consecutive_failures_ = 0;
            ++successful_requests_;
        } else {
            ++consecutive_failures_;
        }
        
        ++total_requests_processed_;
        return result;
        
    } catch (const std::exception& e) {
        ++consecutive_failures_;
        ++total_requests_processed_;
        
        RichPathResult error_result;
        error_result.success = false;
        error_result.algorithmUsed = "exception_occurred";
        
        // Calculate time even for failed requests
        auto end_time = std::chrono::steady_clock::now();
        last_stats_.total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        last_stats_.warnings.push_back("Exception: " + std::string(e.what()));
        
        return error_result;
    }
}

RichPathResult RichPathPlanner::planPoseToPose(const Eigen::Vector3d& start_pose,
                                               const Eigen::Vector3d& goal_pose,
                                               const algorithms::AlgorithmConfig& config) {
    PlanningRequest request;
    request.start_pose = start_pose;
    request.goal_pose = goal_pose;
    request.algorithm_config = config;
    request.request_id = generateRequestId();
    request.timestamp = std::chrono::steady_clock::now();
    
    return planPath(request);
}

RichPathResult RichPathPlanner::planPoseToNode(const Eigen::Vector3d& start_pose,
                                               int32_t goal_node_id,
                                               const algorithms::AlgorithmConfig& config) {
    PlanningRequest request;
    request.start_pose = start_pose;
    request.goal_node_id = goal_node_id;
    request.algorithm_config = config;
    request.request_id = generateRequestId();
    request.timestamp = std::chrono::steady_clock::now();
    
    return planPath(request);
}

RichPathResult RichPathPlanner::planNodeToNode(int32_t start_node_id,
                                               int32_t goal_node_id,
                                               const algorithms::AlgorithmConfig& config) {
    // Get node poses from data loader
    const auto& nodes = data_loader_->getNodes();
    
    auto start_it = nodes.find(start_node_id);
    if (start_it == nodes.end()) {
        RichPathResult error_result;
        error_result.success = false;
        error_result.algorithmUsed = "invalid_start_node";
        return error_result;
    }
    
    PlanningRequest request;
    request.start_pose = Eigen::Vector3d(start_it->second.x, start_it->second.y, start_it->second.theta);
    request.start_node_id = start_node_id;
    request.goal_node_id = goal_node_id;
    request.algorithm_config = config;
    request.request_id = generateRequestId();
    request.timestamp = std::chrono::steady_clock::now();
    
    return planPath(request);
}

// ========================================================================
// ADVANCED PLANNING FEATURES
// ========================================================================

RichPathResult RichPathPlanner::planMultiWaypoint(const Eigen::Vector3d& start_pose,
                                                  const std::vector<int32_t>& waypoint_nodes,
                                                  const std::optional<Eigen::Vector3d>& goal_pose,
                                                  const algorithms::AlgorithmConfig& config) {
    if (waypoint_nodes.empty()) {
        return goal_pose ? planPoseToPose(start_pose, *goal_pose, config) :
                          RichPathResult{}; // Invalid result
    }
    
    // Plan segments between waypoints
    std::vector<RichPathResult> segments;
    Eigen::Vector3d current_pose = start_pose;
    
    // Plan to each waypoint
    for (size_t i = 0; i < waypoint_nodes.size(); ++i) {
        auto segment = planPoseToNode(current_pose, waypoint_nodes[i], config);
        if (!segment.success) {
            return segment; // Return failed result
        }
        
        segments.push_back(segment);
        
        // Update current pose to the end of this segment
        if (!segment.poseSequence.empty()) {
            current_pose = segment.poseSequence.back();
        }
    }
    
    // Plan final segment to goal pose if specified
    if (goal_pose) {
        auto final_segment = planPoseToPose(current_pose, *goal_pose, config);
        if (!final_segment.success) {
            return final_segment; // Return failed result
        }
        segments.push_back(final_segment);
    }
    
    // Combine segments into single result
    RichPathResult combined_result;
    combined_result.success = true;
    combined_result.algorithmUsed = "multi_waypoint";
    
    for (const auto& segment : segments) {
        // Append nodes (avoid duplicates at waypoints)
        if (combined_result.nodeSequence.empty() || 
            combined_result.nodeSequence.back().id != segment.nodeSequence.front().id) {
            combined_result.nodeSequence.insert(
                combined_result.nodeSequence.end(),
                segment.nodeSequence.begin(),
                segment.nodeSequence.end());
        } else {
            combined_result.nodeSequence.insert(
                combined_result.nodeSequence.end(),
                segment.nodeSequence.begin() + 1,
                segment.nodeSequence.end());
        }
        
        // Append links
        combined_result.linkSequence.insert(
            combined_result.linkSequence.end(),
            segment.linkSequence.begin(),
            segment.linkSequence.end());
        
        // Append poses (avoid duplicates)
        if (combined_result.poseSequence.empty() ||
            !combined_result.poseSequence.back().isApprox(segment.poseSequence.front())) {
            combined_result.poseSequence.insert(
                combined_result.poseSequence.end(),
                segment.poseSequence.begin(),
                segment.poseSequence.end());
        } else {
            combined_result.poseSequence.insert(
                combined_result.poseSequence.end(),
                segment.poseSequence.begin() + 1,
                segment.poseSequence.end());
        }
        
        combined_result.totalDistance += segment.totalDistance;
    }
    
    return combined_result;
}

RichPathResult RichPathPlanner::replanFromCurrent(const Eigen::Vector3d& current_pose,
                                                  const RichPathResult& previous_path,
                                                  int32_t goal_node_id,
                                                  const algorithms::AlgorithmConfig& config) {
    PlanningRequest request;
    request.start_pose = current_pose;
    request.goal_node_id = goal_node_id;
    request.algorithm_config = config;
    request.request_id = generateRequestId();
    request.timestamp = std::chrono::steady_clock::now();
    
    PlanningContext context;
    context.previous_path = previous_path;
    context.prefer_path_continuity = true;
    context.path_deviation_penalty = 0.2; // Penalize deviating from previous path
    
    return planPath(request, context);
}

RichPathResult RichPathPlanner::planWithTimeLimit(const PlanningRequest& request,
                                                  std::chrono::milliseconds max_time) {
    PlanningContext context;
    context.max_planning_time = max_time;
    context.deadline = std::chrono::steady_clock::now() + max_time;
    
    return planPath(request, context);
}

// ========================================================================
// ALGORITHM MANAGEMENT
// ========================================================================

std::vector<std::string> RichPathPlanner::getAvailableAlgorithms() const {
    return algorithms::AlgorithmRegistry::getInstance().getAvailableAlgorithms();
}

void RichPathPlanner::setDefaultAlgorithm(const std::string& algorithm_name) {
    auto available = getAvailableAlgorithms();
    if (std::find(available.begin(), available.end(), algorithm_name) != available.end()) {
        default_algorithm_ = algorithm_name;
        // Note: AlgorithmConfig uses algorithm_type, not algorithm_name
        // default_config_.algorithm_type = /* convert algorithm_name to type */;
    } else {
        throw std::invalid_argument("Algorithm '" + algorithm_name + "' is not available");
    }
}

void RichPathPlanner::registerAlgorithm(std::shared_ptr<algorithms::PathPlanningAlgorithm> algorithm) {
    // Note: Need to implement proper registration through AlgorithmRegistry
    // algorithms::AlgorithmRegistry::getInstance().registerAlgorithm(...);
    throw std::runtime_error("registerAlgorithm not yet implemented");
}

// ========================================================================
// STATISTICS AND DIAGNOSTICS
// ========================================================================

std::unordered_map<std::string, double> RichPathPlanner::getAggregatedStats() const {
    std::unordered_map<std::string, double> stats;
    
    stats["total_requests"] = static_cast<double>(total_requests_processed_);
    stats["successful_requests"] = static_cast<double>(successful_requests_);
    stats["success_rate"] = total_requests_processed_ > 0 ? 
        static_cast<double>(successful_requests_) / total_requests_processed_ : 0.0;
    stats["consecutive_failures"] = static_cast<double>(consecutive_failures_);
    
    // Algorithm usage statistics
    for (const auto& [algorithm, count] : algorithm_usage_count_) {
        stats["usage_" + algorithm] = static_cast<double>(count);
    }
    
    // Average timing statistics
    for (const auto& [algorithm, total_time] : algorithm_total_time_) {
        auto usage_it = algorithm_usage_count_.find(algorithm);
        if (usage_it != algorithm_usage_count_.end() && usage_it->second > 0) {
            stats["avg_time_" + algorithm] = static_cast<double>(total_time.count()) / usage_it->second;
        }
    }
    
    return stats;
}

void RichPathPlanner::clearStatistics() {
    last_stats_ = PlanningStatistics{};
    algorithm_usage_count_.clear();
    algorithm_total_time_.clear();
    total_requests_processed_ = 0;
    successful_requests_ = 0;
    consecutive_failures_ = 0;
}

// ========================================================================
// CONFIGURATION MANAGEMENT
// ========================================================================

void RichPathPlanner::setDefaultConfig(const algorithms::AlgorithmConfig& config) {
    default_config_ = config;
}

void RichPathPlanner::setFallbackAlgorithms(const std::vector<std::string>& fallback_algorithms) {
    // Validate that all algorithms are available
    auto available = getAvailableAlgorithms();
    for (const auto& alg : fallback_algorithms) {
        if (std::find(available.begin(), available.end(), alg) == available.end()) {
            throw std::invalid_argument("Fallback algorithm '" + alg + "' is not available");
        }
    }
    
    fallback_algorithms_ = fallback_algorithms;
}

// ========================================================================
// VALIDATION AND HEALTH CHECKS
// ========================================================================

std::vector<std::string> RichPathPlanner::validatePlannerHealth() const {
    std::vector<std::string> issues;
    
    // Check data loader
    if (!data_loader_) {
        issues.push_back("Data loader is null");
        return issues;
    }
    
    // Check if data is loaded
    try {
        auto& nodes = data_loader_->getNodes();
        auto& links = data_loader_->getLinks();
        
        if (nodes.empty()) {
            issues.push_back("No nodes loaded in data loader");
        }
        if (links.empty()) {
            issues.push_back("No links loaded in data loader");
        }
    } catch (const std::exception& e) {
        issues.push_back("Error accessing data loader: " + std::string(e.what()));
    }
    
    // Check algorithm manager
    if (!algorithm_manager_) {
        issues.push_back("Algorithm manager is null");
    } else {
        auto algorithms = algorithms::AlgorithmRegistry::getInstance().getAvailableAlgorithms();
        if (algorithms.empty()) {
            issues.push_back("No algorithms available");
        }
    }
    
    // Check health based on recent performance
    if (consecutive_failures_ > 10) {
        issues.push_back("High consecutive failure count: " + std::to_string(consecutive_failures_));
    }
    
    auto now = std::chrono::steady_clock::now();
    auto time_since_success = std::chrono::duration_cast<std::chrono::minutes>(
        now - last_successful_plan_);
    if (time_since_success > std::chrono::minutes(60)) {
        issues.push_back("No successful planning in over 60 minutes");
    }
    
    return issues;
}

bool RichPathPlanner::runSelfTest() {
    try {
        // Get first two nodes for testing
        const auto& nodes = data_loader_->getNodes();
        if (nodes.size() < 2) {
            return false;
        }
        
        auto it = nodes.begin();
        auto start_node = it->second;
        ++it;
        auto goal_node = it->second;
        
        // Try simple node-to-node planning
        auto test_config = default_config_;
        // Note: AlgorithmConfig uses algorithm_type, not algorithm_name
        // test_config.algorithm_type = /* convert default_algorithm_ to type */;
        
        auto result = planNodeToNode(start_node.id, goal_node.id, test_config);
        return result.success;
        
    } catch (const std::exception&) {
        return false;
    }
}

// ========================================================================
// INTERNAL IMPLEMENTATION
// ========================================================================

RichPathResult RichPathPlanner::executePlanningWorkflow(const PlanningRequest& request,
                                                        const PlanningContext& context) {
    auto workflow_start = std::chrono::steady_clock::now();
    
    // Preprocess request
    auto processed_request = preprocessRequest(request, context);
    auto preprocess_end = std::chrono::steady_clock::now();
    
    // Determine algorithm to use
    std::string algorithm_name = "dijkstra"; // Default, since AlgorithmConfig uses algorithm_type
    if (algorithm_name.empty()) {
        algorithm_name = default_algorithm_;
    }
    
    // Execute algorithm
    auto algorithm_start = std::chrono::steady_clock::now();
    auto result = executeAlgorithm(algorithm_name, processed_request, context);
    auto algorithm_end = std::chrono::steady_clock::now();
    
    // If primary algorithm failed, try fallbacks
    if (!result.success && !fallback_algorithms_.empty()) {
        result = executeFallbackAlgorithms(processed_request, context);
        last_stats_.fallback_used = !result.success || result.algorithmUsed != algorithm_name;
    }
    
    // Postprocess result
    if (result.success) {
        result = postprocessResult(std::move(result), processed_request, context);
    }
    
    auto workflow_end = std::chrono::steady_clock::now();
    
    // Update timing statistics
    last_stats_.total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        workflow_end - workflow_start);
    last_stats_.preprocessing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        preprocess_end - workflow_start);
    last_stats_.algorithm_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        algorithm_end - algorithm_start);
    last_stats_.postprocessing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        workflow_end - algorithm_end);
    
    return result;
}

PlanningRequest RichPathPlanner::preprocessRequest(const PlanningRequest& request,
                                                   const PlanningContext& context) {
    PlanningRequest processed = request;
    
    // Fill in missing node IDs by finding closest nodes
    if (!processed.start_node_id) {
        processed.start_node_id = findClosestNode(processed.start_pose);
    }
    
    if (!processed.goal_node_id && processed.goal_pose) {
        processed.goal_node_id = findClosestNode(*processed.goal_pose);
    }
    
    // Merge default config with request config
    algorithms::AlgorithmConfig merged_config = default_config_;
    // TODO: Implement proper config merging
    // Note: AlgorithmConfig uses algorithm_type, not algorithm_name
    merged_config.algorithm_type = processed.algorithm_config.algorithm_type;
    processed.algorithm_config = merged_config;
    
    return processed;
}

std::optional<int32_t> RichPathPlanner::findClosestNode(const Eigen::Vector3d& pose) {
    const auto& nodes = data_loader_->getNodes();
    if (nodes.empty()) {
        return std::nullopt;
    }
    
    int32_t closest_id = -1;
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& [node_id, node] : nodes) {
        double dx = pose.x() - node.x;
        double dy = pose.y() - node.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_id = node_id;
        }
    }
    
    return closest_id;
}

bool RichPathPlanner::validatePlanningRequest(const PlanningRequest& request) {
    // Check basic validity
    if (!request.isValid()) {
        last_stats_.warnings.push_back("Invalid planning request");
        return false;
    }
    
    // Check that we have either goal pose or goal node
    if (!request.goal_pose && !request.goal_node_id) {
        last_stats_.warnings.push_back("No goal specified");
        return false;
    }
    
    // Check that goal node exists if specified
    if (request.goal_node_id) {
        const auto& nodes = data_loader_->getNodes();
        if (nodes.find(*request.goal_node_id) == nodes.end()) {
            last_stats_.warnings.push_back("Goal node " + std::to_string(*request.goal_node_id) + " not found");
            return false;
        }
    }
    
    return true;
}

RichPathResult RichPathPlanner::executeAlgorithm(const std::string& algorithm_name,
                                                 const PlanningRequest& request,
                                                 const PlanningContext& context) {
    try {
        // For now, create a simple mock implementation
        // TODO: Integrate with actual algorithm manager
        
        RichPathResult result;
        result.success = true;
        result.algorithmUsed = algorithm_name;
        result.totalDistance = 0.0;
        
        // Mock path from start to goal
        if (request.start_node_id && request.goal_node_id) {
            const auto& nodes = data_loader_->getNodes();
            
            auto start_it = nodes.find(*request.start_node_id);
            auto goal_it = nodes.find(*request.goal_node_id);
            
            if (start_it != nodes.end() && goal_it != nodes.end()) {
                result.nodeSequence = {start_it->second, goal_it->second};
                result.poseSequence = {
                    Eigen::Vector3d(start_it->second.x, start_it->second.y, start_it->second.theta),
                    Eigen::Vector3d(goal_it->second.x, goal_it->second.y, goal_it->second.theta)
                };
                
                double dx = goal_it->second.x - start_it->second.x;
                double dy = goal_it->second.y - start_it->second.y;
                result.totalDistance = std::sqrt(dx * dx + dy * dy);
            }
        }
        
        return result;
        
    } catch (const std::exception& e) {
        RichPathResult error_result;
        error_result.success = false;
        error_result.algorithmUsed = algorithm_name + "_failed";
        last_stats_.warnings.push_back("Algorithm " + algorithm_name + " failed: " + e.what());
        return error_result;
    }
}

RichPathResult RichPathPlanner::executeFallbackAlgorithms(const PlanningRequest& request,
                                                          const PlanningContext& context) {
    for (const auto& fallback_alg : fallback_algorithms_) {
        try {
            auto result = executeAlgorithm(fallback_alg, request, context);
            if (result.success) {
                result.algorithmUsed += "_fallback";
                return result;
            }
        } catch (const std::exception&) {
            // Continue to next fallback
            continue;
        }
    }
    
    // All fallbacks failed
    RichPathResult error_result;
    error_result.success = false;
    error_result.algorithmUsed = "all_fallbacks_failed";
    return error_result;
}

RichPathResult RichPathPlanner::postprocessResult(RichPathResult result,
                                                  const PlanningRequest& request,
                                                  const PlanningContext& context) {
    // Validate result
    validateResult(result);
    
    // Calculate additional metrics
    last_stats_.path_length = result.totalDistance;
    last_stats_.path_segments = result.poseSequence.size();
    last_stats_.path_smoothness = calculatePathSmoothness(result);
    last_stats_.direction_changes = countDirectionChanges(result);
    
    return result;
}

void RichPathPlanner::validateResult(const RichPathResult& result) {
    if (!result.success) {
        return; // No validation needed for failed results
    }
    
    if (result.nodeSequence.empty()) {
        last_stats_.warnings.push_back("Result has no nodes");
    }
    
    if (result.poseSequence.empty()) {
        last_stats_.warnings.push_back("Result has no poses");
    }
    
    if (result.totalDistance < 0) {
        last_stats_.warnings.push_back("Negative total distance");
    }
}

void RichPathPlanner::startProfiling() {
    profiling_start_ = std::chrono::steady_clock::now();
    last_stats_ = PlanningStatistics{}; // Reset stats
}

void RichPathPlanner::stopProfiling() {
    // Profiling data is collected throughout the planning process
    // This is just a placeholder for any final profiling steps
}

void RichPathPlanner::updateStatistics(const RichPathResult& result,
                                      const std::string& algorithm_used) {
    last_stats_.algorithm_used = algorithm_used;
    
    // Update algorithm usage counts
    ++algorithm_usage_count_[algorithm_used];
    algorithm_total_time_[algorithm_used] += last_stats_.algorithm_time;
    
    // Update global counters based on success/failure
    // (handled in planPath method)
}

double RichPathPlanner::calculatePathSmoothness(const RichPathResult& result) {
    if (result.poseSequence.size() < 3) {
        return 1.0; // Perfectly smooth for straight line
    }
    
    double total_curvature = 0.0;
    size_t curvature_points = 0;
    
    for (size_t i = 1; i < result.poseSequence.size() - 1; ++i) {
        const auto& p1 = result.poseSequence[i - 1];
        const auto& p2 = result.poseSequence[i];
        const auto& p3 = result.poseSequence[i + 1];
        
        // Calculate curvature using three points
        double dx1 = p2.x() - p1.x();
        double dy1 = p2.y() - p1.y();
        double dx2 = p3.x() - p2.x();
        double dy2 = p3.y() - p2.y();
        
        double cross_product = dx1 * dy2 - dy1 * dx2;
        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (len1 > 0 && len2 > 0) {
            double curvature = std::abs(cross_product) / (len1 * len2);
            total_curvature += curvature;
            ++curvature_points;
        }
    }
    
    if (curvature_points == 0) {
        return 1.0;
    }
    
    double avg_curvature = total_curvature / curvature_points;
    return 1.0 / (1.0 + avg_curvature); // Normalize to [0, 1] where 1 is smoothest
}

size_t RichPathPlanner::countDirectionChanges(const RichPathResult& result) {
    if (result.poseSequence.size() < 2) {
        return 0;
    }
    
    size_t direction_changes = 0;
    double prev_angle = 0.0;
    bool first_segment = true;
    
    for (size_t i = 1; i < result.poseSequence.size(); ++i) {
        const auto& p1 = result.poseSequence[i - 1];
        const auto& p2 = result.poseSequence[i];
        
        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();
        double angle = std::atan2(dy, dx);
        
        if (!first_segment) {
            double angle_diff = std::abs(angle - prev_angle);
            if (angle_diff > M_PI) {
                angle_diff = 2 * M_PI - angle_diff; // Handle wrap-around
            }
            
            if (angle_diff > M_PI / 4) { // > 45 degrees considered a direction change
                ++direction_changes;
            }
        }
        
        prev_angle = angle;
        first_segment = false;
    }
    
    return direction_changes;
}

std::string RichPathPlanner::generateRequestId() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(100000, 999999);
    
    auto now = std::chrono::steady_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::stringstream ss;
    ss << "req_" << timestamp << "_" << dis(gen);
    return ss.str();
}

} // namespace core
} // namespace vrobot_route_follow