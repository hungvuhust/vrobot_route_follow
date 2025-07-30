#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

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
 * @brief Validation severity levels
 */
enum class ValidationSeverity {
    INFO,        // Informational message
    WARNING,     // Non-critical issue that should be addressed
    ERROR,       // Critical issue that prevents safe execution
    CRITICAL     // Fatal issue that requires immediate attention
};

/**
 * @brief Validation issue structure
 */
struct ValidationIssue {
    ValidationSeverity severity;
    std::string category;           // e.g., "connectivity", "geometry", "safety"
    std::string description;        // Human-readable description
    std::string code;              // Machine-readable code (e.g., "PATH_001")
    
    // Location information
    std::optional<size_t> segment_index;      // Which path segment has the issue
    std::optional<int32_t> node_id;           // Related node ID (if applicable)
    std::optional<int32_t> link_id;           // Related link ID (if applicable)
    
    // Suggested fixes
    std::vector<std::string> suggested_fixes;
    
    // Additional context
    std::unordered_map<std::string, double> metrics;
};

/**
 * @brief Validation configuration
 */
struct ValidationConfig {
    // Basic validation settings
    bool enable_connectivity_check = true;
    bool enable_geometry_check = true;
    bool enable_safety_check = true;
    bool enable_feasibility_check = true;
    bool enable_consistency_check = true;
    
    // Geometric constraints
    double max_segment_length = 10.0;          // Maximum allowed segment length (m)
    double min_segment_length = 0.01;          // Minimum allowed segment length (m)
    double max_turn_angle = M_PI;              // Maximum allowed turn angle (rad)
    double max_curvature = 10.0;               // Maximum allowed curvature (1/m)
    
    // Continuity tolerances
    double position_tolerance = 0.1;           // Position continuity tolerance (m)
    double orientation_tolerance = 0.1;        // Orientation continuity tolerance (rad)
    double velocity_tolerance = 0.5;           // Velocity continuity tolerance (m/s)
    
    // Safety constraints
    double min_clearance = 0.2;                // Minimum required clearance (m)
    double max_acceleration = 3.0;             // Maximum allowed acceleration (m/s²)
    double max_jerk = 5.0;                     // Maximum allowed jerk (m/s³)
    
    // Graph connectivity validation
    bool require_valid_nodes = true;           // All nodes must exist in graph
    bool require_valid_links = true;           // All links must exist in graph
    bool allow_disconnected_segments = false;  // Allow gaps in path
    
    // Performance constraints
    bool enable_detailed_analysis = true;      // Perform detailed geometric analysis
    size_t max_validation_issues = 100;       // Maximum number of issues to report
    
    // Custom validation functions
    std::vector<std::function<std::vector<ValidationIssue>(const RichPathResult&)>> custom_validators;
};

/**
 * @brief Validation result structure
 */
struct ValidationResult {
    bool is_valid = false;                     // Overall validation status
    bool is_safe = false;                      // Safety validation status
    bool is_feasible = false;                  // Feasibility validation status
    
    // Issue categorization
    size_t total_issues = 0;
    size_t critical_issues = 0;
    size_t error_issues = 0;
    size_t warning_issues = 0;
    size_t info_issues = 0;
    
    // Detailed issues
    std::vector<ValidationIssue> issues;
    
    // Performance metrics
    std::unordered_map<std::string, double> metrics;
    
    // Summary information
    std::string summary;
    std::vector<std::string> recommendations;
    
    // Validation context
    std::string validator_version;
    std::chrono::steady_clock::time_point validation_timestamp;
    std::chrono::milliseconds validation_time{0};
};

/**
 * @brief Robot constraints for validation
 */
struct RobotValidationConstraints {
    // Physical dimensions
    double width = 0.6;                        // Robot width (m)
    double length = 1.0;                       // Robot length (m)
    double height = 0.5;                       // Robot height (m)
    
    // Kinematic constraints
    double max_linear_velocity = 2.0;          // Maximum linear velocity (m/s)
    double max_angular_velocity = 1.0;         // Maximum angular velocity (rad/s)
    double max_linear_acceleration = 2.0;      // Maximum linear acceleration (m/s²)
    double max_angular_acceleration = 2.0;     // Maximum angular acceleration (rad/s²)
    double max_jerk = 5.0;                     // Maximum jerk (m/s³)
    
    // Safety margins
    double safety_margin = 0.2;               // Additional safety clearance (m)
    double emergency_stop_distance = 1.5;     // Required emergency stop distance (m)
    
    // Turn constraints
    double min_turn_radius = 0.3;             // Minimum turning radius (m)
    double max_turn_rate = 0.8;               // Maximum turn rate (rad/s)
    
    // Stability constraints
    double max_lateral_acceleration = 1.5;    // Maximum lateral acceleration (m/s²)
    double stability_margin = 0.1;            // Stability safety margin
};

/**
 * @brief Comprehensive path validator for rich path results
 * 
 * This class provides extensive validation capabilities:
 * - Graph connectivity validation
 * - Geometric consistency checks
 * - Kinematic feasibility analysis
 * - Safety constraint verification
 * - Performance quality assessment
 * - Custom validation rule support
 */
class RichPathValidator {
public:
    // Constructor and destructor
    RichPathValidator() = default;
    ~RichPathValidator() = default;

    // Copy/move semantics
    RichPathValidator(const RichPathValidator&) = default;
    RichPathValidator& operator=(const RichPathValidator&) = default;
    RichPathValidator(RichPathValidator&&) = default;
    RichPathValidator& operator=(RichPathValidator&&) = default;

    // ========================================================================
    // PRIMARY VALIDATION INTERFACE
    // ========================================================================

    /**
     * @brief Comprehensive path validation
     * @param path Path to validate
     * @param config Validation configuration
     * @param constraints Robot constraints (optional)
     * @return Detailed validation result
     */
    ValidationResult validatePath(const RichPathResult& path,
                                 const ValidationConfig& config = ValidationConfig{},
                                 const std::optional<RobotValidationConstraints>& constraints = std::nullopt);

    /**
     * @brief Quick validation with default settings
     * @param path Path to validate
     * @return Basic validation result
     */
    ValidationResult quickValidate(const RichPathResult& path);

    /**
     * @brief Safety-focused validation
     * @param path Path to validate
     * @param constraints Robot constraints
     * @return Safety validation result
     */
    ValidationResult validateSafety(const RichPathResult& path,
                                   const RobotValidationConstraints& constraints);

    // ========================================================================
    // SPECIFIC VALIDATION METHODS
    // ========================================================================

    /**
     * @brief Validate graph connectivity
     * @param path Path to validate
     * @param nodes Available nodes map
     * @param links Available links map
     * @return Connectivity validation issues
     */
    std::vector<ValidationIssue> validateConnectivity(const RichPathResult& path,
                                                     const std::unordered_map<int32_t, NodeInfo>& nodes,
                                                     const std::unordered_map<int32_t, LinkInfo>& links);

    /**
     * @brief Validate geometric consistency
     * @param path Path to validate
     * @param config Validation configuration
     * @return Geometry validation issues
     */
    std::vector<ValidationIssue> validateGeometry(const RichPathResult& path,
                                                 const ValidationConfig& config);

    /**
     * @brief Validate kinematic feasibility
     * @param path Path to validate
     * @param constraints Robot constraints
     * @return Feasibility validation issues
     */
    std::vector<ValidationIssue> validateKinematics(const RichPathResult& path,
                                                   const RobotValidationConstraints& constraints);

    /**
     * @brief Validate path continuity
     * @param path Path to validate
     * @param config Validation configuration
     * @return Continuity validation issues
     */
    std::vector<ValidationIssue> validateContinuity(const RichPathResult& path,
                                                   const ValidationConfig& config);

    /**
     * @brief Validate velocity profile
     * @param path Path to validate
     * @param constraints Robot constraints
     * @return Velocity validation issues
     */
    std::vector<ValidationIssue> validateVelocityProfile(const RichPathResult& path,
                                                        const RobotValidationConstraints& constraints);

    // ========================================================================
    // ADVANCED VALIDATION FEATURES
    // ========================================================================

    /**
     * @brief Validate path against custom rules
     * @param path Path to validate
     * @param custom_rules Vector of custom validation functions
     * @return Custom validation issues
     */
    std::vector<ValidationIssue> validateWithCustomRules(const RichPathResult& path,
                                                        const std::vector<std::function<std::vector<ValidationIssue>(const RichPathResult&)>>& custom_rules);

    /**
     * @brief Batch validation of multiple paths
     * @param paths Vector of paths to validate
     * @param config Validation configuration
     * @return Vector of validation results
     */
    std::vector<ValidationResult> batchValidate(const std::vector<RichPathResult>& paths,
                                               const ValidationConfig& config = ValidationConfig{});

    /**
     * @brief Validate path segment by segment
     * @param path Path to validate
     * @param config Validation configuration
     * @return Segment-wise validation results
     */
    std::vector<ValidationResult> segmentWiseValidation(const RichPathResult& path,
                                                       const ValidationConfig& config = ValidationConfig{});

    // ========================================================================
    // ISSUE ANALYSIS AND REPORTING
    // ========================================================================

    /**
     * @brief Get validation summary statistics
     * @param result Validation result to analyze
     * @return Summary statistics
     */
    std::unordered_map<std::string, double> getValidationStatistics(const ValidationResult& result);

    /**
     * @brief Generate human-readable validation report
     * @param result Validation result
     * @param include_details Include detailed issue descriptions
     * @return Formatted validation report
     */
    std::string generateValidationReport(const ValidationResult& result,
                                        bool include_details = true);

    /**
     * @brief Filter issues by severity
     * @param issues Vector of validation issues
     * @param min_severity Minimum severity level
     * @return Filtered issues
     */
    std::vector<ValidationIssue> filterIssuesBySeverity(const std::vector<ValidationIssue>& issues,
                                                       ValidationSeverity min_severity);

    /**
     * @brief Group issues by category
     * @param issues Vector of validation issues
     * @return Issues grouped by category
     */
    std::unordered_map<std::string, std::vector<ValidationIssue>> groupIssuesByCategory(const std::vector<ValidationIssue>& issues);

    // ========================================================================
    // CONFIGURATION MANAGEMENT
    // ========================================================================

    /**
     * @brief Set default validation configuration
     * @param config Default configuration
     */
    void setDefaultConfig(const ValidationConfig& config);

    /**
     * @brief Get current default configuration
     * @return Current default configuration
     */
    const ValidationConfig& getDefaultConfig() const { return default_config_; }

    /**
     * @brief Set default robot constraints
     * @param constraints Default robot constraints
     */
    void setDefaultRobotConstraints(const RobotValidationConstraints& constraints);

    /**
     * @brief Get current default robot constraints
     * @return Current default robot constraints
     */
    const RobotValidationConstraints& getDefaultRobotConstraints() const { return default_constraints_; }

    /**
     * @brief Register custom validation rule
     * @param rule_name Name of the custom rule
     * @param validator Custom validation function
     */
    void registerCustomRule(const std::string& rule_name,
                           std::function<std::vector<ValidationIssue>(const RichPathResult&)> validator);

    // ========================================================================
    // STATISTICS AND DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get aggregated validation statistics
     * @return Aggregated statistics over all validations
     */
    std::unordered_map<std::string, double> getAggregatedStats() const;

    /**
     * @brief Clear all statistics
     */
    void clearStatistics();

private:
    // ========================================================================
    // INTERNAL VALIDATION METHODS
    // ========================================================================

    // Core validation algorithms
    std::vector<ValidationIssue> validateBasicPath(const RichPathResult& path);
    std::vector<ValidationIssue> validateNodeSequence(const std::vector<NodeInfo>& nodes);
    std::vector<ValidationIssue> validateLinkSequence(const std::vector<LinkInfo>& links);
    std::vector<ValidationIssue> validatePoseSequence(const std::vector<Eigen::Vector3d>& poses,
                                                     const ValidationConfig& config);
    
    // Specific validation checks
    std::vector<ValidationIssue> checkSegmentLengths(const std::vector<Eigen::Vector3d>& poses,
                                                    const ValidationConfig& config);
    std::vector<ValidationIssue> checkTurnAngles(const std::vector<Eigen::Vector3d>& poses,
                                                const ValidationConfig& config);
    std::vector<ValidationIssue> checkCurvature(const std::vector<Eigen::Vector3d>& poses,
                                               const ValidationConfig& config);
    std::vector<ValidationIssue> checkAcceleration(const std::vector<Eigen::Vector3d>& poses,
                                                  const std::vector<LinkInfo>& links,
                                                  const RobotValidationConstraints& constraints);
    
    // Utility methods
    double calculateSegmentLength(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    double calculateTurnAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
    double calculateCurvature(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
    double calculateAcceleration(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3,
                                double dt1, double dt2);
    
    // Issue creation helpers
    ValidationIssue createIssue(ValidationSeverity severity,
                               const std::string& category,
                               const std::string& description,
                               const std::string& code);
    
    ValidationResult compileValidationResult(const std::vector<ValidationIssue>& issues);
    
    // Statistics updating
    void updateValidationStatistics(const ValidationResult& result);

    // Member variables
    ValidationConfig default_config_;
    RobotValidationConstraints default_constraints_;
    
    // Custom validation rules
    std::unordered_map<std::string, std::function<std::vector<ValidationIssue>(const RichPathResult&)>> custom_rules_;
    
    // Statistics tracking
    size_t total_validations_ = 0;
    size_t total_issues_found_ = 0;
    size_t total_paths_rejected_ = 0;
    std::unordered_map<std::string, size_t> issue_category_counts_;
    std::unordered_map<ValidationSeverity, size_t> severity_counts_;
    std::chrono::milliseconds total_validation_time_{0};
};

} // namespace core
} // namespace vrobot_route_follow