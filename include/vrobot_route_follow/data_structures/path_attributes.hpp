#pragma once

#include <map>
#include <optional>
#include <string>
#include <variant>

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Flexible path attributes for future extensibility
 * 
 * This structure allows adding any path-related attributes without
 * breaking existing code. Attributes can be velocity, acceleration,
 * curvature, lane width, traffic rules, etc.
 */
struct PathAttributes {
  // ========================================================================
  // VELOCITY ATTRIBUTES
  // ========================================================================
  
  /// Maximum velocity for this path segment
  std::optional<double> max_velocity;
  
  /// Minimum velocity for this path segment  
  std::optional<double> min_velocity;
  
  /// Target velocity for this path segment
  std::optional<double> target_velocity;
  
  /// Maximum acceleration
  std::optional<double> max_acceleration;
  
  /// Maximum deceleration
  std::optional<double> max_deceleration;
  
  // ========================================================================
  // GEOMETRIC ATTRIBUTES
  // ========================================================================
  
  /// Path width (for narrow passages)
  std::optional<double> path_width;
  
  /// Maximum curvature (for sharp turns)
  std::optional<double> max_curvature;
  
  /// Path type (normal, narrow, curve, intersection, etc.)
  std::optional<std::string> path_type;
  
  // ========================================================================
  // TRAFFIC ATTRIBUTES
  // ========================================================================
  
  /// Traffic direction (forward, backward, both)
  std::optional<std::string> traffic_direction;
  
  /// Priority level (1=lowest, 10=highest)
  std::optional<int> priority_level;
  
  /// Is emergency path
  std::optional<bool> is_emergency_path;
  
  /// Requires special permissions
  std::optional<bool> requires_permission;
  
  // ========================================================================
  // ZONE ATTRIBUTES
  // ========================================================================
  
  /// Zone type (pickup, dropoff, charging, waiting, etc.)
  std::optional<std::string> zone_type;
  
  /// Maximum waiting time in zone (seconds)
  std::optional<double> max_waiting_time;
  
  /// Requires specific orientation
  std::optional<bool> requires_orientation;
  
  // ========================================================================
  // DYNAMIC ATTRIBUTES SYSTEM
  // ========================================================================
  
  /// Generic attribute storage for future extensions
  /// Supports: double, int, string, bool
  using AttributeValue = std::variant<double, int, std::string, bool>;
  std::map<std::string, AttributeValue> dynamic_attributes;
  
  // ========================================================================
  // UTILITY METHODS
  // ========================================================================
  
  /**
   * @brief Set a dynamic attribute
   */
  template<typename T>
  void setAttribute(const std::string& key, const T& value) {
    dynamic_attributes[key] = value;
  }
  
  /**
   * @brief Get a dynamic attribute
   */
  template<typename T>
  std::optional<T> getAttribute(const std::string& key) const {
    auto it = dynamic_attributes.find(key);
    if (it != dynamic_attributes.end()) {
      if (std::holds_alternative<T>(it->second)) {
        return std::get<T>(it->second);
      }
    }
    return std::nullopt;
  }
  
  /**
   * @brief Check if attribute exists
   */
  bool hasAttribute(const std::string& key) const {
    return dynamic_attributes.find(key) != dynamic_attributes.end();
  }
  
  /**
   * @brief Get effective velocity (with fallback logic)
   */
  double getEffectiveVelocity(double default_velocity = 0.5) const {
    if (target_velocity.has_value()) {
      return target_velocity.value();
    }
    if (max_velocity.has_value()) {
      return max_velocity.value();
    }
    return default_velocity;
  }
  
  /**
   * @brief Merge with another PathAttributes (other takes precedence)
   */
  PathAttributes merge(const PathAttributes& other) const {
    PathAttributes result = *this;
    
    // Merge specific attributes
    if (other.max_velocity.has_value()) result.max_velocity = other.max_velocity;
    if (other.min_velocity.has_value()) result.min_velocity = other.min_velocity;
    if (other.target_velocity.has_value()) result.target_velocity = other.target_velocity;
    if (other.max_acceleration.has_value()) result.max_acceleration = other.max_acceleration;
    if (other.max_deceleration.has_value()) result.max_deceleration = other.max_deceleration;
    if (other.path_width.has_value()) result.path_width = other.path_width;
    if (other.max_curvature.has_value()) result.max_curvature = other.max_curvature;
    if (other.path_type.has_value()) result.path_type = other.path_type;
    if (other.traffic_direction.has_value()) result.traffic_direction = other.traffic_direction;
    if (other.priority_level.has_value()) result.priority_level = other.priority_level;
    if (other.is_emergency_path.has_value()) result.is_emergency_path = other.is_emergency_path;
    if (other.requires_permission.has_value()) result.requires_permission = other.requires_permission;
    if (other.zone_type.has_value()) result.zone_type = other.zone_type;
    if (other.max_waiting_time.has_value()) result.max_waiting_time = other.max_waiting_time;
    if (other.requires_orientation.has_value()) result.requires_orientation = other.requires_orientation;
    
    // Merge dynamic attributes
    for (const auto& [key, value] : other.dynamic_attributes) {
      result.dynamic_attributes[key] = value;
    }
    
    return result;
  }
  
  /**
   * @brief Convert to string for debugging
   */
  std::string toString() const {
    std::string result = "PathAttributes{";
    
    if (max_velocity.has_value()) {
      result += "max_vel:" + std::to_string(max_velocity.value()) + " ";
    }
    if (target_velocity.has_value()) {
      result += "target_vel:" + std::to_string(target_velocity.value()) + " ";
    }
    if (path_type.has_value()) {
      result += "type:" + path_type.value() + " ";
    }
    if (zone_type.has_value()) {
      result += "zone:" + zone_type.value() + " ";
    }
    
    if (!dynamic_attributes.empty()) {
      result += "dynamic:" + std::to_string(dynamic_attributes.size()) + " ";
    }
    
    result += "}";
    return result;
  }
  
  /**
   * @brief Convert to JSON string
   */
  std::string toJsonString() const;
  
  /**
   * @brief Create from JSON string
   */
  static PathAttributes fromJsonString(const std::string& json_str);
};

} // namespace data_structures
} // namespace vrobot_route_follow