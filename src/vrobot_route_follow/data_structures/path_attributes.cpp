#include "vrobot_route_follow/data_structures/path_attributes.hpp"
#include <sstream>
#include <stdexcept>

namespace vrobot_route_follow {
namespace data_structures {

// ========================================================================
// SERIALIZATION METHODS
// ========================================================================

std::string PathAttributes::toJsonString() const {
  std::ostringstream oss;
  oss << "{";
  
  bool first = true;
  
  // Helper lambda to add comma if not first
  auto addComma = [&]() {
    if (!first) oss << ",";
    first = false;
  };
  
  // Velocity attributes
  if (max_velocity.has_value()) {
    addComma();
    oss << "\"max_velocity\":" << max_velocity.value();
  }
  
  if (min_velocity.has_value()) {
    addComma();
    oss << "\"min_velocity\":" << min_velocity.value();
  }
  
  if (target_velocity.has_value()) {
    addComma();
    oss << "\"target_velocity\":" << target_velocity.value();
  }
  
  if (max_acceleration.has_value()) {
    addComma();
    oss << "\"max_acceleration\":" << max_acceleration.value();
  }
  
  if (max_deceleration.has_value()) {
    addComma();
    oss << "\"max_deceleration\":" << max_deceleration.value();
  }
  
  // Geometric attributes
  if (path_width.has_value()) {
    addComma();
    oss << "\"path_width\":" << path_width.value();
  }
  
  if (max_curvature.has_value()) {
    addComma();
    oss << "\"max_curvature\":" << max_curvature.value();
  }
  
  if (path_type.has_value()) {
    addComma();
    oss << "\"path_type\":\"" << path_type.value() << "\"";
  }
  
  // Traffic attributes
  if (traffic_direction.has_value()) {
    addComma();
    oss << "\"traffic_direction\":\"" << traffic_direction.value() << "\"";
  }
  
  if (priority_level.has_value()) {
    addComma();
    oss << "\"priority_level\":" << priority_level.value();
  }
  
  if (is_emergency_path.has_value()) {
    addComma();
    oss << "\"is_emergency_path\":" << (is_emergency_path.value() ? "true" : "false");
  }
  
  if (requires_permission.has_value()) {
    addComma();
    oss << "\"requires_permission\":" << (requires_permission.value() ? "true" : "false");
  }
  
  // Zone attributes
  if (zone_type.has_value()) {
    addComma();
    oss << "\"zone_type\":\"" << zone_type.value() << "\"";
  }
  
  if (max_waiting_time.has_value()) {
    addComma();
    oss << "\"max_waiting_time\":" << max_waiting_time.value();
  }
  
  if (requires_orientation.has_value()) {
    addComma();
    oss << "\"requires_orientation\":" << (requires_orientation.value() ? "true" : "false");
  }
  
  // Dynamic attributes
  if (!dynamic_attributes.empty()) {
    addComma();
    oss << "\"dynamic_attributes\":{";
    bool first_dynamic = true;
    for (const auto& [key, value] : dynamic_attributes) {
      if (!first_dynamic) oss << ",";
      oss << "\"" << key << "\":";
      
      // Handle different variant types
      std::visit([&](const auto& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::string>) {
          oss << "\"" << v << "\"";
        } else if constexpr (std::is_same_v<T, bool>) {
          oss << (v ? "true" : "false");
        } else {
          oss << v;
        }
      }, value);
      
      first_dynamic = false;
    }
    oss << "}";
  }
  
  oss << "}";
  return oss.str();
}

PathAttributes PathAttributes::fromJsonString(const std::string& json_str) {
  // Simple JSON parsing - in production, should use proper JSON library
  PathAttributes result;
  
  // This is a simplified implementation
  // In a real implementation, you would use a proper JSON parser like nlohmann/json
  
  // For now, throw an exception to indicate this needs proper implementation
  throw std::runtime_error(
    "PathAttributes::fromJsonString not fully implemented - needs proper JSON parser");
  
  return result;
}

} // namespace data_structures
} // namespace vrobot_route_follow