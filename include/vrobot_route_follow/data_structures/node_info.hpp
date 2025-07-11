#pragma once

#include <cstdint>

#include <Eigen/Dense>
#include <cmath>
#include <optional>
#include <string>

// Forward declarations
namespace drogon_model {
namespace amr_01 {
namespace amr_ros2 {
class Node;
}
} // namespace amr_01
} // namespace drogon_model

namespace vrobot_route_follow {
namespace data_structures {

/**
 * @brief Rich node information structure containing all database attributes
 *
 * This struct replaces the template-based approach and contains all node
 * attributes from the database, making it easy to extend and trace paths.
 */
struct NodeInfo {
  // Core database fields
  int32_t     id;
  std::string node_name;
  float       x, y, theta;
  std::string type;
  int32_t     map_id;

  // Extended attributes for future use
  std::optional<double>      max_waiting_time;
  std::optional<std::string> zone_type;
  std::optional<std::string> description;
  std::optional<bool>        is_charging_station;
  std::optional<double>      priority;

  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================

  /**
   * @brief Default constructor
   */
  NodeInfo() = default;

  /**
   * @brief Constructor from database values
   */
  NodeInfo(int32_t id, const std::string &node_name, float x, float y,
           float theta, const std::string &type, int32_t map_id)
      : id(id), node_name(node_name), x(x), y(y), theta(theta), type(type),
        map_id(map_id) {}

  /**
   * @brief Constructor from database ORM object
   */
  explicit NodeInfo(const drogon_model::amr_01::amr_ros2::Node &db_node);

  // ========================================================================
  // CONVERSION METHODS
  // ========================================================================

  /**
   * @brief Convert to Eigen pose representation
   */
  Eigen::Vector3d toPose() const { return Eigen::Vector3d(x, y, theta); }

  /**
   * @brief Get 2D position as Eigen vector
   */
  Eigen::Vector2d getPosition() const { return Eigen::Vector2d(x, y); }

  /**
   * @brief Convert to JSON string for serialization
   */
  std::string toJsonString() const;

  /**
   * @brief Create from JSON string
   */
  static NodeInfo fromJsonString(const std::string &json_str);

  // ========================================================================
  // UTILITY METHODS
  // ========================================================================

  /**
   * @brief Calculate distance to another node
   */
  double distanceTo(const NodeInfo &other) const {
    Eigen::Vector2d diff = getPosition() - other.getPosition();
    return diff.norm();
  }

  /**
   * @brief Calculate distance to a pose
   */
  double distanceTo(const Eigen::Vector3d &pose) const {
    Eigen::Vector2d diff = getPosition() - pose.head<2>();
    return diff.norm();
  }

  /**
   * @brief Calculate distance to a 2D position
   */
  double distanceTo(const Eigen::Vector2d &position) const {
    return (getPosition() - position).norm();
  }

  /**
   * @brief Check if this is a special node type
   */
  bool isChargingStation() const {
    return is_charging_station.value_or(false) || type == "charging" ||
           type == "charge_station";
  }

  bool isWaitingPoint() const {
    return type == "waiting" || type == "wait_point";
  }

  bool isPickupPoint() const {
    return type == "pickup" || type == "pick_point";
  }

  bool isDropoffPoint() const {
    return type == "dropoff" || type == "drop_point";
  }

  // ========================================================================
  // OPERATORS
  // ========================================================================

  bool operator==(const NodeInfo &other) const {
    return id == other.id && map_id == other.map_id;
  }

  bool operator!=(const NodeInfo &other) const { return !(*this == other); }

  // For use in hash maps
  struct Hash {
    std::size_t operator()(const NodeInfo &node) const {
      return std::hash<int32_t>()(node.id) ^
             (std::hash<int32_t>()(node.map_id) << 1);
    }
  };

  // ========================================================================
  // DEBUG/LOGGING
  // ========================================================================

  /**
   * @brief Get string representation for debugging
   */
  std::string toString() const {
    return "NodeInfo{id=" + std::to_string(id) + ", name='" + node_name +
           "', pos=(" + std::to_string(x) + "," + std::to_string(y) + "," +
           std::to_string(theta) + "), type='" + type + "'}";
  }

  /**
   * @brief Get short string representation
   */
  std::string toShortString() const {
    return node_name.empty() ? std::to_string(id) : node_name;
  }
};

} // namespace data_structures
} // namespace vrobot_route_follow

// Hash specialization for std::unordered_map
namespace std {
template <> struct hash<vrobot_route_follow::data_structures::NodeInfo> {
  std::size_t
  operator()(const vrobot_route_follow::data_structures::NodeInfo &node) const {
    return vrobot_route_follow::data_structures::NodeInfo::Hash{}(node);
  }
};
} // namespace std