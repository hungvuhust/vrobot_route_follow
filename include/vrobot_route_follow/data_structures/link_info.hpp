#pragma once

#include "node_info.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

// Forward declarations
namespace drogon_model {
namespace amr_01 {
namespace amr_ros2 {
class Straightlink;
}
} // namespace amr_01
} // namespace drogon_model

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Rich link information structure containing all database attributes
 *
 * This struct replaces the template-based approach and contains all link
 * attributes from the database, making it easy to extend and trace paths.
 */
struct LinkInfo {
  // Core database fields
  int32_t id_straight_link;
  int32_t id_start;
  int32_t id_end;
  int32_t map_id;
  double  max_velocity;

  // Extended attributes for future use
  std::optional<std::string> link_type;
  std::optional<double>      width;
  std::optional<bool>        bidirectional;
  std::optional<double>      min_velocity;
  std::optional<std::string> traffic_direction; // "forward", "backward", "both"
  std::optional<std::string> zone_restriction;
  std::optional<bool>        is_emergency_path;
  std::optional<double>      priority;

  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================

  /**
   * @brief Default constructor
   */
  LinkInfo() = default;

  /**
   * @brief Constructor from database values
   */
  LinkInfo(int32_t id_straight_link, int32_t id_start, int32_t id_end,
           int32_t map_id, double max_velocity)
      : id_straight_link(id_straight_link), id_start(id_start), id_end(id_end),
        map_id(map_id), max_velocity(max_velocity) {}

  /**
   * @brief Constructor from database ORM object
   */
  explicit LinkInfo(
      const drogon_model::amr_01::amr_ros2::Straightlink &db_link);

  // ========================================================================
  // WEIGHT CALCULATION METHODS
  // ========================================================================

  /**
   * @brief Calculate weight based on Euclidean distance
   */
  double calculateWeight(const NodeInfo &start_node,
                         const NodeInfo &end_node) const {
    return start_node.distanceTo(end_node);
  }

  /**
   * @brief Calculate weight with velocity consideration
   */
  double calculateTimeWeight(const NodeInfo &start_node,
                             const NodeInfo &end_node) const {
    double distance = calculateWeight(start_node, end_node);
    double velocity =
        max_velocity > 0 ? max_velocity : 1.0; // Avoid division by zero
    return distance / velocity;
  }

  /**
   * @brief Calculate weight with custom factors
   */
  double calculateCustomWeight(const NodeInfo &start_node,
                               const NodeInfo &end_node,
                               double          distance_factor = 1.0,
                               double          velocity_factor = 0.0,
                               double          priority_factor = 0.0) const {
    double distance = calculateWeight(start_node, end_node);
    double time_cost =
        velocity_factor > 0 ? calculateTimeWeight(start_node, end_node) : 0.0;
    double priority_cost =
        priority_factor > 0 ? (priority.value_or(1.0) * priority_factor) : 0.0;

    return distance * distance_factor + time_cost + priority_cost;
  }

  // ========================================================================
  // UTILITY METHODS
  // ========================================================================

  /**
   * @brief Check if link allows traffic in given direction
   */
  bool allowsTraffic(bool forward_direction = true) const {
    if (!bidirectional.has_value() || bidirectional.value()) {
      return true; // Default is bidirectional
    }

    if (traffic_direction.has_value()) {
      const std::string &direction = traffic_direction.value();
      if (direction == "both")
        return true;
      if (direction == "forward" && forward_direction)
        return true;
      if (direction == "backward" && !forward_direction)
        return true;
      return false;
    }

    return true; // Default allow
  }

  /**
   * @brief Check if this is an emergency path
   */
  bool isEmergencyPath() const { return is_emergency_path.value_or(false); }

  /**
   * @brief Get effective velocity considering constraints
   */
  double getEffectiveVelocity() const {
    if (min_velocity.has_value()) {
      return std::min(max_velocity,
                      std::max(min_velocity.value(), max_velocity));
    }
    return max_velocity;
  }

  /**
   * @brief Check if link has width restrictions
   */
  bool hasWidthRestriction() const { return width.has_value(); }

  /**
   * @brief Get link width (default if not specified)
   */
  double getWidth(double default_width = 1.0) const {
    return width.value_or(default_width);
  }

  // ========================================================================
  // CONVERSION METHODS
  // ========================================================================

  /**
   * @brief Convert to JSON string for serialization
   */
  std::string toJsonString() const;

  /**
   * @brief Create from JSON string
   */
  static LinkInfo fromJsonString(const std::string &json_str);

  // ========================================================================
  // OPERATORS
  // ========================================================================

  bool operator==(const LinkInfo &other) const {
    return id_straight_link == other.id_straight_link && map_id == other.map_id;
  }

  bool operator!=(const LinkInfo &other) const { return !(*this == other); }

  // For use in hash maps
  struct Hash {
    std::size_t operator()(const LinkInfo &link) const {
      return std::hash<int32_t>()(link.id_straight_link) ^
             (std::hash<int32_t>()(link.map_id) << 1);
    }
  };

  // ========================================================================
  // DEBUG/LOGGING
  // ========================================================================

  /**
   * @brief Get string representation for debugging
   */
  std::string toString() const {
    return "LinkInfo{id=" + std::to_string(id_straight_link) +
           ", start=" + std::to_string(id_start) +
           ", end=" + std::to_string(id_end) +
           ", max_vel=" + std::to_string(max_velocity) + ", type='" +
           link_type.value_or("default") + "'}";
  }

  /**
   * @brief Get short string representation
   */
  std::string toShortString() const {
    return std::to_string(id_start) + "->" + std::to_string(id_end);
  }

  /**
   * @brief Get direction arrow for visualization
   */
  std::string getDirectionArrow() const {
    if (bidirectional.value_or(true)) {
      return "<->";
    } else if (traffic_direction.has_value()) {
      const std::string &dir = traffic_direction.value();
      if (dir == "forward")
        return "-->";
      if (dir == "backward")
        return "<--";
      if (dir == "both")
        return "<->";
    }
    return "-->";
  }
};

} // namespace data_structures
} // namespace vrobot_route_follow

// Hash specialization for std::unordered_map
namespace std {
template <> struct hash<vrobot_route_follow::data_structures::LinkInfo> {
  std::size_t
  operator()(const vrobot_route_follow::data_structures::LinkInfo &link) const {
    return vrobot_route_follow::data_structures::LinkInfo::Hash{}(link);
  }
};
} // namespace std