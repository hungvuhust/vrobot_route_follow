#include "vrobot_route_follow/data_structures/link_info.hpp"
#include <sstream>
#include <stdexcept>

namespace vrobot_route_follow {
namespace data_structures {

// Note: Constructor from database ORM is implemented in database_converter.cpp
// to avoid circular dependencies and keep the data structures clean

// ========================================================================
// SERIALIZATION METHODS
// ========================================================================

std::string LinkInfo::toJsonString() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"id_straight_link\":" << id_straight_link << ",";
  oss << "\"id_start\":" << id_start << ",";
  oss << "\"id_end\":" << id_end << ",";
  oss << "\"map_id\":" << map_id << ",";
  oss << "\"max_velocity\":" << max_velocity;

  // Extended attributes
  if (link_type.has_value()) {
    oss << ",\"link_type\":\"" << link_type.value() << "\"";
  }

  if (width.has_value()) {
    oss << ",\"width\":" << width.value();
  }

  if (bidirectional.has_value()) {
    oss << ",\"bidirectional\":" << (bidirectional.value() ? "true" : "false");
  }

  if (min_velocity.has_value()) {
    oss << ",\"min_velocity\":" << min_velocity.value();
  }

  if (traffic_direction.has_value()) {
    oss << ",\"traffic_direction\":\"" << traffic_direction.value() << "\"";
  }

  if (zone_restriction.has_value()) {
    oss << ",\"zone_restriction\":\"" << zone_restriction.value() << "\"";
  }

  if (is_emergency_path.has_value()) {
    oss << ",\"is_emergency_path\":"
        << (is_emergency_path.value() ? "true" : "false");
  }

  if (priority.has_value()) {
    oss << ",\"priority\":" << priority.value();
  }

  oss << "}";
  return oss.str();
}

LinkInfo LinkInfo::fromJsonString(const std::string &json_str) {
  // Simple JSON parsing - in production, should use proper JSON library
  LinkInfo link_info;

  // This is a simplified implementation
  // In a real implementation, you would use a proper JSON parser like
  // nlohmann/json

  // For now, throw an exception to indicate this needs proper implementation
  throw std::runtime_error(
      "LinkInfo::fromJsonString not fully implemented - needs proper JSON "
      "parser");

  return link_info;
}

} // namespace data_structures
} // namespace vrobot_route_follow