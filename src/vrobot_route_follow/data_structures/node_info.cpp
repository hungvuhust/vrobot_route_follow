#include "vrobot_route_follow/data_structures/node_info.hpp"
#include <sstream>
#include <stdexcept>

namespace vrobot_route_follow {
namespace data_structures {

// Note: Constructor from database ORM is implemented in database_converter.cpp
// to avoid circular dependencies and keep the data structures clean

// ========================================================================
// SERIALIZATION METHODS
// ========================================================================

std::string NodeInfo::toJsonString() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"id\":" << id << ",";
  oss << "\"node_name\":\"" << node_name << "\",";
  oss << "\"x\":" << x << ",";
  oss << "\"y\":" << y << ",";
  oss << "\"theta\":" << theta << ",";
  oss << "\"type\":\"" << type << "\",";
  oss << "\"map_id\":" << map_id;

  // Extended attributes
  if (max_waiting_time.has_value()) {
    oss << ",\"max_waiting_time\":" << max_waiting_time.value();
  }

  if (zone_type.has_value()) {
    oss << ",\"zone_type\":\"" << zone_type.value() << "\"";
  }

  if (description.has_value()) {
    oss << ",\"description\":\"" << description.value() << "\"";
  }

  if (is_charging_station.has_value()) {
    oss << ",\"is_charging_station\":"
        << (is_charging_station.value() ? "true" : "false");
  }

  if (priority.has_value()) {
    oss << ",\"priority\":" << priority.value();
  }

  oss << "}";
  return oss.str();
}

NodeInfo NodeInfo::fromJsonString(const std::string &json_str) {
  // Simple JSON parsing - in production, should use proper JSON library
  NodeInfo node_info;

  // This is a simplified implementation
  // In a real implementation, you would use a proper JSON parser like
  // nlohmann/json

  // For now, throw an exception to indicate this needs proper implementation
  throw std::runtime_error(
      "NodeInfo::fromJsonString not fully implemented - needs proper JSON "
      "parser");

  return node_info;
}

} // namespace data_structures
} // namespace vrobot_route_follow