#include "vrobot_route_follow/core/rich_path_validator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace vrobot_route_follow {
namespace core {

// ========================================================================
// PRIMARY VALIDATION INTERFACE
// ========================================================================

ValidationResult RichPathValidator::validatePath(const RichPathResult& path,
                                                 const ValidationConfig& config,
                                                 const std::optional<RobotValidationConstraints>& constraints) {
    auto start_time = std::chrono::steady_clock::now();
    
    std::vector<ValidationIssue> all_issues;
    
    // Basic path validation
    auto basic_issues = validateBasicPath(path);
    all_issues.insert(all_issues.end(), basic_issues.begin(), basic_issues.end());
    
    if (!path.success) {
        ValidationResult result = compileValidationResult(all_issues);
        result.summary = "Path is marked as unsuccessful";
        return result;
    }
    
    // Geometry validation
    if (config.enable_geometry_check) {
        auto geom_issues = validateGeometry(path, config);
        all_issues.insert(all_issues.end(), geom_issues.begin(), geom_issues.end());
    }
    
    // Kinematics validation
    if (config.enable_feasibility_check && constraints) {
        auto kinematic_issues = validateKinematics(path, *constraints);
        all_issues.insert(all_issues.end(), kinematic_issues.begin(), kinematic_issues.end());
    }
    
    // Continuity validation
    if (config.enable_consistency_check) {
        auto continuity_issues = validateContinuity(path, config);
        all_issues.insert(all_issues.end(), continuity_issues.begin(), continuity_issues.end());
    }
    
    // Custom validation rules
    if (!config.custom_validators.empty()) {
        auto custom_issues = validateWithCustomRules(path, config.custom_validators);
        all_issues.insert(all_issues.end(), custom_issues.begin(), custom_issues.end());
    }
    
    // Limit number of issues reported
    if (all_issues.size() > config.max_validation_issues) {
        all_issues.resize(config.max_validation_issues);
    }
    
    ValidationResult result = compileValidationResult(all_issues);
    
    // Calculate validation time
    auto end_time = std::chrono::steady_clock::now();
    result.validation_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    result.validation_timestamp = start_time;
    
    // Update statistics
    updateValidationStatistics(result);
    
    return result;
}

ValidationResult RichPathValidator::quickValidate(const RichPathResult& path) {
    ValidationConfig quick_config;
    quick_config.enable_detailed_analysis = false;
    quick_config.max_validation_issues = 10;
    
    return validatePath(path, quick_config);
}

ValidationResult RichPathValidator::validateSafety(const RichPathResult& path,
                                                   const RobotValidationConstraints& constraints) {
    ValidationConfig safety_config;
    safety_config.enable_safety_check = true;
    safety_config.enable_feasibility_check = true;
    safety_config.enable_geometry_check = true;
    safety_config.enable_connectivity_check = false;
    safety_config.enable_consistency_check = false;
    
    return validatePath(path, safety_config, constraints);
}

// ========================================================================
// SPECIFIC VALIDATION METHODS
// ========================================================================

std::vector<ValidationIssue> RichPathValidator::validateGeometry(const RichPathResult& path,
                                                                const ValidationConfig& config) {
    std::vector<ValidationIssue> issues;
    
    if (path.poseSequence.empty()) {
        issues.push_back(createIssue(ValidationSeverity::ERROR, "geometry", 
                                   "Path has no pose sequence", "GEOM_001"));
        return issues;
    }
    
    // Check segment lengths
    auto length_issues = checkSegmentLengths(path.poseSequence, config);
    issues.insert(issues.end(), length_issues.begin(), length_issues.end());
    
    // Check turn angles
    auto turn_issues = checkTurnAngles(path.poseSequence, config);
    issues.insert(issues.end(), turn_issues.begin(), turn_issues.end());
    
    // Check curvature
    if (config.enable_detailed_analysis) {
        auto curvature_issues = checkCurvature(path.poseSequence, config);
        issues.insert(issues.end(), curvature_issues.begin(), curvature_issues.end());
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::validateKinematics(const RichPathResult& path,
                                                                  const RobotValidationConstraints& constraints) {
    std::vector<ValidationIssue> issues;
    
    // Check velocity constraints
    for (const auto& link : path.linkSequence) {
        if (link.max_velocity > constraints.max_linear_velocity) {
            ValidationIssue issue = createIssue(ValidationSeverity::ERROR, "kinematics",
                                              "Link velocity exceeds robot maximum", "KIN_001");
            issue.link_id = link.id_straight_link;
            issue.metrics["max_velocity"] = link.max_velocity;
            issue.metrics["constraint_limit"] = constraints.max_linear_velocity;
            issues.push_back(issue);
        }
    }
    
    // Check acceleration constraints
    if (path.poseSequence.size() >= 3) {
        auto accel_issues = checkAcceleration(path.poseSequence, path.linkSequence, constraints);
        issues.insert(issues.end(), accel_issues.begin(), accel_issues.end());
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::validateContinuity(const RichPathResult& path,
                                                                  const ValidationConfig& config) {
    std::vector<ValidationIssue> issues;
    
    if (path.poseSequence.size() < 2) {
        return issues;
    }
    
    // Check position continuity
    for (size_t i = 1; i < path.poseSequence.size(); ++i) {
        double distance = calculateSegmentLength(path.poseSequence[i-1], path.poseSequence[i]);
        
        if (distance < config.min_segment_length) {
            ValidationIssue issue = createIssue(ValidationSeverity::WARNING, "continuity",
                                              "Very short segment detected", "CONT_001");
            issue.segment_index = i - 1;
            issue.metrics["segment_length"] = distance;
            issue.metrics["minimum_required"] = config.min_segment_length;
            issues.push_back(issue);
        }
        
        if (distance > config.max_segment_length) {
            ValidationIssue issue = createIssue(ValidationSeverity::WARNING, "continuity",
                                              "Very long segment detected", "CONT_002");
            issue.segment_index = i - 1;
            issue.metrics["segment_length"] = distance;
            issue.metrics["maximum_allowed"] = config.max_segment_length;
            issues.push_back(issue);
        }
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::validateWithCustomRules(const RichPathResult& path,
                                                                       const std::vector<std::function<std::vector<ValidationIssue>(const RichPathResult&)>>& custom_rules) {
    std::vector<ValidationIssue> issues;
    
    for (const auto& rule : custom_rules) {
        try {
            auto rule_issues = rule(path);
            issues.insert(issues.end(), rule_issues.begin(), rule_issues.end());
        } catch (const std::exception& e) {
            ValidationIssue issue = createIssue(ValidationSeverity::ERROR, "custom",
                                              "Custom validation rule failed: " + std::string(e.what()), "CUSTOM_001");
            issues.push_back(issue);
        }
    }
    
    return issues;
}

// ========================================================================
// ISSUE ANALYSIS AND REPORTING
// ========================================================================

std::unordered_map<std::string, double> RichPathValidator::getValidationStatistics(const ValidationResult& result) {
    std::unordered_map<std::string, double> stats;
    
    stats["total_issues"] = static_cast<double>(result.total_issues);
    stats["critical_issues"] = static_cast<double>(result.critical_issues);
    stats["error_issues"] = static_cast<double>(result.error_issues);
    stats["warning_issues"] = static_cast<double>(result.warning_issues);
    stats["info_issues"] = static_cast<double>(result.info_issues);
    stats["validation_time_ms"] = static_cast<double>(result.validation_time.count());
    stats["is_valid"] = result.is_valid ? 1.0 : 0.0;
    stats["is_safe"] = result.is_safe ? 1.0 : 0.0;
    stats["is_feasible"] = result.is_feasible ? 1.0 : 0.0;
    
    return stats;
}

std::string RichPathValidator::generateValidationReport(const ValidationResult& result,
                                                       bool include_details) {
    std::stringstream report;
    
    report << "=== PATH VALIDATION REPORT ===\n";
    report << "Overall Status: " << (result.is_valid ? "VALID" : "INVALID") << "\n";
    report << "Safety Status: " << (result.is_safe ? "SAFE" : "UNSAFE") << "\n";
    report << "Feasibility Status: " << (result.is_feasible ? "FEASIBLE" : "INFEASIBLE") << "\n";
    report << "Total Issues: " << result.total_issues << "\n";
    report << "  - Critical: " << result.critical_issues << "\n";
    report << "  - Errors: " << result.error_issues << "\n";
    report << "  - Warnings: " << result.warning_issues << "\n";
    report << "  - Info: " << result.info_issues << "\n";
    report << "Validation Time: " << result.validation_time.count() << " ms\n";
    
    if (!result.summary.empty()) {
        report << "Summary: " << result.summary << "\n";
    }
    
    if (include_details && !result.issues.empty()) {
        report << "\n=== DETAILED ISSUES ===\n";
        for (size_t i = 0; i < result.issues.size(); ++i) {
            const auto& issue = result.issues[i];
            report << "[" << i + 1 << "] ";
            
            switch (issue.severity) {
                case ValidationSeverity::CRITICAL: report << "CRITICAL"; break;
                case ValidationSeverity::ERROR: report << "ERROR"; break;
                case ValidationSeverity::WARNING: report << "WARNING"; break;
                case ValidationSeverity::INFO: report << "INFO"; break;
            }
            
            report << " (" << issue.category << "/" << issue.code << "): " << issue.description << "\n";
            
            if (issue.segment_index) {
                report << "  Segment: " << *issue.segment_index << "\n";
            }
            if (issue.node_id) {
                report << "  Node ID: " << *issue.node_id << "\n";
            }
            if (issue.link_id) {
                report << "  Link ID: " << *issue.link_id << "\n";
            }
        }
    }
    
    if (!result.recommendations.empty()) {
        report << "\n=== RECOMMENDATIONS ===\n";
        for (const auto& rec : result.recommendations) {
            report << "- " << rec << "\n";
        }
    }
    
    return report.str();
}

// ========================================================================
// CONFIGURATION MANAGEMENT
// ========================================================================

void RichPathValidator::setDefaultConfig(const ValidationConfig& config) {
    default_config_ = config;
}

void RichPathValidator::setDefaultRobotConstraints(const RobotValidationConstraints& constraints) {
    default_constraints_ = constraints;
}

void RichPathValidator::registerCustomRule(const std::string& rule_name,
                                          std::function<std::vector<ValidationIssue>(const RichPathResult&)> validator) {
    custom_rules_[rule_name] = validator;
}

// ========================================================================
// STATISTICS AND DIAGNOSTICS
// ========================================================================

std::unordered_map<std::string, double> RichPathValidator::getAggregatedStats() const {
    std::unordered_map<std::string, double> stats;
    
    stats["total_validations"] = static_cast<double>(total_validations_);
    stats["total_issues_found"] = static_cast<double>(total_issues_found_);
    stats["total_paths_rejected"] = static_cast<double>(total_paths_rejected_);
    stats["rejection_rate"] = total_validations_ > 0 ? 
        static_cast<double>(total_paths_rejected_) / total_validations_ : 0.0;
    stats["avg_validation_time_ms"] = total_validations_ > 0 ?
        static_cast<double>(total_validation_time_.count()) / total_validations_ : 0.0;
    
    // Category statistics
    for (const auto& [category, count] : issue_category_counts_) {
        stats["issues_" + category] = static_cast<double>(count);
    }
    
    return stats;
}

void RichPathValidator::clearStatistics() {
    total_validations_ = 0;
    total_issues_found_ = 0;
    total_paths_rejected_ = 0;
    issue_category_counts_.clear();
    severity_counts_.clear();
    total_validation_time_ = std::chrono::milliseconds{0};
}

// ========================================================================
// INTERNAL VALIDATION METHODS
// ========================================================================

std::vector<ValidationIssue> RichPathValidator::validateBasicPath(const RichPathResult& path) {
    std::vector<ValidationIssue> issues;
    
    if (!path.success) {
        issues.push_back(createIssue(ValidationSeverity::ERROR, "basic", 
                                   "Path planning was unsuccessful", "BASIC_001"));
    }
    
    if (path.poseSequence.empty()) {
        issues.push_back(createIssue(ValidationSeverity::ERROR, "basic", 
                                   "Path has no poses", "BASIC_002"));
    }
    
    if (path.totalDistance < 0) {
        issues.push_back(createIssue(ValidationSeverity::ERROR, "basic", 
                                   "Path has negative total distance", "BASIC_003"));
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::checkSegmentLengths(const std::vector<Eigen::Vector3d>& poses,
                                                                   const ValidationConfig& config) {
    std::vector<ValidationIssue> issues;
    
    for (size_t i = 1; i < poses.size(); ++i) {
        double length = calculateSegmentLength(poses[i-1], poses[i]);
        
        if (length > config.max_segment_length) {
            ValidationIssue issue = createIssue(ValidationSeverity::WARNING, "geometry",
                                              "Segment exceeds maximum length", "GEOM_002");
            issue.segment_index = i - 1;
            issue.metrics["segment_length"] = length;
            issue.metrics["max_allowed"] = config.max_segment_length;
            issues.push_back(issue);
        }
        
        if (length < config.min_segment_length) {
            ValidationIssue issue = createIssue(ValidationSeverity::INFO, "geometry",
                                              "Very short segment", "GEOM_003");
            issue.segment_index = i - 1;
            issue.metrics["segment_length"] = length;
            issue.metrics["min_expected"] = config.min_segment_length;
            issues.push_back(issue);
        }
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::checkTurnAngles(const std::vector<Eigen::Vector3d>& poses,
                                                               const ValidationConfig& config) {
    std::vector<ValidationIssue> issues;
    
    if (poses.size() < 3) {
        return issues;
    }
    
    for (size_t i = 1; i < poses.size() - 1; ++i) {
        double turn_angle = calculateTurnAngle(poses[i-1], poses[i], poses[i+1]);
        
        if (turn_angle > config.max_turn_angle) {
            ValidationIssue issue = createIssue(ValidationSeverity::ERROR, "geometry",
                                              "Turn angle exceeds maximum", "GEOM_004");
            issue.segment_index = i;
            issue.metrics["turn_angle"] = turn_angle;
            issue.metrics["max_allowed"] = config.max_turn_angle;
            issues.push_back(issue);
        }
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::checkCurvature(const std::vector<Eigen::Vector3d>& poses,
                                                              const ValidationConfig& config) {
    std::vector<ValidationIssue> issues;
    
    if (poses.size() < 3) {
        return issues;
    }
    
    for (size_t i = 1; i < poses.size() - 1; ++i) {
        double curvature = calculateCurvature(poses[i-1], poses[i], poses[i+1]);
        
        if (curvature > config.max_curvature) {
            ValidationIssue issue = createIssue(ValidationSeverity::WARNING, "geometry",
                                              "High curvature detected", "GEOM_005");
            issue.segment_index = i;
            issue.metrics["curvature"] = curvature;
            issue.metrics["max_allowed"] = config.max_curvature;
            issues.push_back(issue);
        }
    }
    
    return issues;
}

std::vector<ValidationIssue> RichPathValidator::checkAcceleration(const std::vector<Eigen::Vector3d>& poses,
                                                                 const std::vector<LinkInfo>& links,
                                                                 const RobotValidationConstraints& constraints) {
    std::vector<ValidationIssue> issues;
    
    // Simplified acceleration check
    // TODO: Implement more sophisticated acceleration analysis
    
    return issues;
}

// ========================================================================
// UTILITY METHODS
// ========================================================================

double RichPathValidator::calculateSegmentLength(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return (p2 - p1).head<2>().norm();
}

double RichPathValidator::calculateTurnAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
    Eigen::Vector2d v1 = (p2 - p1).head<2>().normalized();
    Eigen::Vector2d v2 = (p3 - p2).head<2>().normalized();
    
    double dot_product = v1.dot(v2);
    dot_product = std::clamp(dot_product, -1.0, 1.0);
    
    return std::acos(dot_product);
}

double RichPathValidator::calculateCurvature(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
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

ValidationIssue RichPathValidator::createIssue(ValidationSeverity severity,
                                              const std::string& category,
                                              const std::string& description,
                                              const std::string& code) {
    ValidationIssue issue;
    issue.severity = severity;
    issue.category = category;
    issue.description = description;
    issue.code = code;
    return issue;
}

ValidationResult RichPathValidator::compileValidationResult(const std::vector<ValidationIssue>& issues) {
    ValidationResult result;
    result.issues = issues;
    result.total_issues = issues.size();
    
    // Count issues by severity
    for (const auto& issue : issues) {
        switch (issue.severity) {
            case ValidationSeverity::CRITICAL:
                ++result.critical_issues;
                break;
            case ValidationSeverity::ERROR:
                ++result.error_issues;
                break;
            case ValidationSeverity::WARNING:
                ++result.warning_issues;
                break;
            case ValidationSeverity::INFO:
                ++result.info_issues;
                break;
        }
    }
    
    // Determine overall validity
    result.is_valid = (result.critical_issues == 0 && result.error_issues == 0);
    result.is_safe = (result.critical_issues == 0);
    result.is_feasible = (result.critical_issues == 0 && result.error_issues == 0);
    
    // Generate summary
    if (result.is_valid) {
        result.summary = "Path validation passed";
    } else {
        result.summary = "Path validation failed with " + std::to_string(result.critical_issues + result.error_issues) + " critical issues";
    }
    
    return result;
}

void RichPathValidator::updateValidationStatistics(const ValidationResult& result) {
    ++total_validations_;
    total_issues_found_ += result.total_issues;
    total_validation_time_ += result.validation_time;
    
    if (!result.is_valid) {
        ++total_paths_rejected_;
    }
    
    // Update category counts
    for (const auto& issue : result.issues) {
        ++issue_category_counts_[issue.category];
        ++severity_counts_[issue.severity];
    }
}

} // namespace core
} // namespace vrobot_route_follow