#include "vrobot_route_follow/core/rich_database_loader.hpp"
#include "vrobot_route_follow/utils/database_converter.hpp"

// Drogon ORM includes
#include "models/Map.h"
#include "models/Node.h"
#include "models/Straightlink.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <set>

namespace vrobot_route_follow {
namespace core {

RichDatabaseLoader::RichDatabaseLoader(std::shared_ptr<drogon::orm::DbClient> db_client)
    : db_client_(std::move(db_client)) {
    if (!db_client_) {
        throw std::invalid_argument("Database client cannot be null");
    }
}

// ========================================================================
// PRIMARY LOADING INTERFACE
// ========================================================================

LoadStatistics RichDatabaseLoader::loadMap(const std::string& map_name, 
                                          const DatabaseLoadConfig& config) {
    auto start_time = std::chrono::steady_clock::now();
    
    // Check cache first if enabled
    if (config.enable_cache) {
        auto cached_data = getCachedData(map_name);
        if (cached_data && isCacheValid(*cached_data, config)) {
            // Update access time
            cached_data->last_accessed = std::chrono::steady_clock::now();
            cache_[map_name] = std::move(*cached_data);
            
            // Update current references
            auto& current_cache = cache_[map_name];
            current_nodes_ = &current_cache.nodes;
            current_links_ = &current_cache.links;
            current_adjacency_list_ = &current_cache.adjacency_list;
            current_map_name_ = map_name;
            current_map_id_ = current_cache.load_stats.map_id;
            
            // Update statistics
            ++cache_hits_;
            auto cached_stats = current_cache.load_stats;
            cached_stats.cache_hit = true;
            cached_stats.load_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time);
            
            last_load_stats_ = cached_stats;
            return cached_stats;
        }
    }
    
    // Load from database
    ++cache_misses_;
    return loadFromDatabase(map_name, config);
}

const std::unordered_map<int32_t, NodeInfo>& RichDatabaseLoader::getNodes() const {
    if (!current_nodes_) {
        throw std::runtime_error("No map data loaded. Call loadMap() first.");
    }
    return *current_nodes_;
}

const std::unordered_map<int32_t, LinkInfo>& RichDatabaseLoader::getLinks() const {
    if (!current_links_) {
        throw std::runtime_error("No map data loaded. Call loadMap() first.");
    }
    return *current_links_;
}

const std::unordered_map<int32_t, CurveLinkInfo>& RichDatabaseLoader::getCurvedLinks() const {
    if (!current_curved_links_) {
        throw std::runtime_error("No map data loaded. Call loadMap() first.");
    }
    return *current_curved_links_;
}

const std::unordered_map<int32_t, std::vector<int32_t>>& RichDatabaseLoader::getAdjacencyList() const {
    if (!current_adjacency_list_) {
        throw std::runtime_error("No map data loaded. Call loadMap() first.");
    }
    return *current_adjacency_list_;
}

// ========================================================================
// CACHE MANAGEMENT
// ========================================================================

bool RichDatabaseLoader::isMapCached(const std::string& map_name) const {
    auto cached_data = getCachedData(map_name);
    return cached_data && isCacheValid(*cached_data, default_config_);
}

LoadStatistics RichDatabaseLoader::reloadMap(const std::string& map_name,
                                            const DatabaseLoadConfig& config) {
    // Remove from cache first
    clearMapCache(map_name);
    
    // Load fresh from database
    return loadFromDatabase(map_name, config);
}

void RichDatabaseLoader::clearMapCache(const std::string& map_name) {
    cache_.erase(map_name);
    
    // Reset current pointers if this was the current map
    if (current_map_name_ == map_name) {
        current_nodes_ = nullptr;
        current_links_ = nullptr;
        current_adjacency_list_ = nullptr;
        current_map_name_.clear();
        current_map_id_ = -1;
    }
}

void RichDatabaseLoader::clearAllCache() {
    cache_.clear();
    current_nodes_ = nullptr;
    current_links_ = nullptr;
    current_curved_links_ = nullptr;
    current_adjacency_list_ = nullptr;
    current_map_name_.clear();
    current_map_id_ = -1;
    cache_hits_ = 0;
    cache_misses_ = 0;
}

std::unordered_map<std::string, double> RichDatabaseLoader::getCacheStatistics() const {
    std::unordered_map<std::string, double> stats;
    
    size_t total_requests = cache_hits_ + cache_misses_;
    stats["cache_hit_rate"] = total_requests > 0 ? 
        static_cast<double>(cache_hits_) / total_requests : 0.0;
    stats["cache_size"] = static_cast<double>(cache_.size());
    stats["cache_hits"] = static_cast<double>(cache_hits_);
    stats["cache_misses"] = static_cast<double>(cache_misses_);
    
    return stats;
}

// ========================================================================
// FILTERED LOADING
// ========================================================================

LoadStatistics RichDatabaseLoader::loadMapInBounds(const std::string& map_name,
                                                  const std::pair<double, double>& min_coords,
                                                  const std::pair<double, double>& max_coords) {
    DatabaseLoadConfig config = default_config_;
    config.bounding_box_min = min_coords;
    config.bounding_box_max = max_coords;
    config.enable_cache = false; // Disable cache for filtered loads
    
    return loadMap(map_name, config);
}

LoadStatistics RichDatabaseLoader::loadMapByNodeTypes(const std::string& map_name,
                                                     const std::vector<std::string>& node_types) {
    DatabaseLoadConfig config = default_config_;
    config.node_types_filter = node_types;
    config.enable_cache = false; // Disable cache for filtered loads
    
    return loadMap(map_name, config);
}

// ========================================================================
// STATUS AND DIAGNOSTICS
// ========================================================================

bool RichDatabaseLoader::isDatabaseConnected() const {
    if (!db_client_) {
        return false;
    }
    
    try {
        // Try a simple query to test connection
        auto result = db_client_->execSqlSync("SELECT 1");
        return result.size() > 0;
    } catch (const std::exception&) {
        return false;
    }
}

std::vector<std::string> RichDatabaseLoader::validateDataIntegrity() const {
    std::vector<std::string> issues;
    
    if (!current_nodes_ || !current_links_ || !current_adjacency_list_) {
        issues.push_back("No data loaded");
        return issues;
    }
    
    // Check for orphaned links
    for (const auto& [link_id, link] : *current_links_) {
        if (current_nodes_->find(link.id_start) == current_nodes_->end()) {
            issues.push_back("Link " + std::to_string(link_id) + 
                            " references non-existent start node " + std::to_string(link.id_start));
        }
        if (current_nodes_->find(link.id_end) == current_nodes_->end()) {
            issues.push_back("Link " + std::to_string(link_id) + 
                            " references non-existent end node " + std::to_string(link.id_end));
        }
    }
    
    // Check adjacency list consistency
    for (const auto& [node_id, link_ids] : *current_adjacency_list_) {
        if (current_nodes_->find(node_id) == current_nodes_->end()) {
            issues.push_back("Adjacency list contains non-existent node " + std::to_string(node_id));
            continue;
        }
        
        for (const auto& link_id : link_ids) {
            if (current_links_->find(link_id) == current_links_->end()) {
                issues.push_back("Adjacency list references non-existent link " + std::to_string(link_id));
            }
        }
    }
    
    return issues;
}

// ========================================================================
// INTERNAL IMPLEMENTATION
// ========================================================================

LoadStatistics RichDatabaseLoader::loadFromDatabase(const std::string& map_name, 
                                                   const DatabaseLoadConfig& config) {
    auto start_time = std::chrono::steady_clock::now();
    LoadStatistics stats;
    stats.map_name = map_name;
    stats.cache_hit = false;
    
    try {
        // Get map ID
        auto map_id_opt = getMapId(map_name);
        if (!map_id_opt) {
            throw std::runtime_error("Map '" + map_name + "' not found in database");
        }
        stats.map_id = *map_id_opt;
        
        // Load nodes and links
        auto nodes_vec = loadNodes(stats.map_id, config);
        auto links_vec = loadLinks(stats.map_id, config);
        
        // Apply filters if specified
        applyFilters(nodes_vec, links_vec, config);
        
        stats.nodes_loaded = nodes_vec.size();
        stats.links_loaded = links_vec.size();
        
        // Create cached data structure
        CachedMapData cached_data;
        cached_data.loaded_at = std::chrono::steady_clock::now();
        cached_data.last_accessed = cached_data.loaded_at;
        
        // Convert to maps
        for (auto&& node : nodes_vec) {
            cached_data.nodes[node.id] = std::move(node);
        }
        for (auto&& link : links_vec) {
            cached_data.links[link.id_straight_link] = std::move(link);
        }
        
        // Build adjacency list
        buildAdjacencyList(nodes_vec, links_vec, cached_data.adjacency_list);
        
        cached_data.load_stats = stats;
        
        // Update cache if enabled
        if (config.enable_cache) {
            updateCache(map_name, std::move(cached_data));
            
            // Update current references to cached data
            auto& current_cache = cache_[map_name];
            current_nodes_ = &current_cache.nodes;
            current_links_ = &current_cache.links;
            current_adjacency_list_ = &current_cache.adjacency_list;
        } else {
            // Store in temporary cache entry (will be cleaned up later)
            cache_[map_name] = std::move(cached_data);
            auto& current_cache = cache_[map_name];
            current_nodes_ = &current_cache.nodes;
            current_links_ = &current_cache.links;
            current_adjacency_list_ = &current_cache.adjacency_list;
        }
        
        current_map_name_ = map_name;
        current_map_id_ = stats.map_id;
        
        // Calculate load time
        auto end_time = std::chrono::steady_clock::now();
        stats.load_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        last_load_stats_ = stats;
        
    } catch (const std::exception& e) {
        stats.nodes_loaded = 0;
        stats.links_loaded = 0;
        auto end_time = std::chrono::steady_clock::now();
        stats.load_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        // Re-throw with context
        throw std::runtime_error("Failed to load map '" + map_name + "': " + e.what());
    }
    
    return stats;
}

std::optional<int32_t> RichDatabaseLoader::getMapId(const std::string& map_name) {
    try {
        drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Map> mapper(db_client_);
        auto criteria = drogon::orm::Criteria(
            drogon_model::amr_01::amr_ros2::Map::Cols::_map_name,
            drogon::orm::CompareOperator::EQ,
            map_name
        );
        
        auto maps = mapper.findBy(criteria);
        if (maps.empty()) {
            return std::nullopt;
        }
        
        return *maps[0].getIdMap();
    } catch (const std::exception& e) {
        std::cerr << "Error getting map ID for '" << map_name << "': " << e.what() << std::endl;
        return std::nullopt;
    }
}

std::vector<NodeInfo> RichDatabaseLoader::loadNodes(int32_t map_id, const DatabaseLoadConfig& config) {
    std::vector<NodeInfo> nodes;
    
    try {
        drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Node> mapper(db_client_);
        auto criteria = drogon::orm::Criteria(
            drogon_model::amr_01::amr_ros2::Node::Cols::_map_id,
            drogon::orm::CompareOperator::EQ,
            map_id
        );
        
        auto db_nodes = mapper.findBy(criteria);
        nodes.reserve(db_nodes.size());
        
        for (const auto& db_node : db_nodes) {
            nodes.push_back(utils::DatabaseConverter::convertNode(db_node));
        }
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load nodes: " + std::string(e.what()));
    }
    
    return nodes;
}

std::vector<LinkInfo> RichDatabaseLoader::loadLinks(int32_t map_id, const DatabaseLoadConfig& config) {
    std::vector<LinkInfo> links;
    
    try {
        drogon::orm::Mapper<drogon_model::amr_01::amr_ros2::Straightlink> mapper(db_client_);
        auto criteria = drogon::orm::Criteria(
            drogon_model::amr_01::amr_ros2::Straightlink::Cols::_map_id,
            drogon::orm::CompareOperator::EQ,
            map_id
        );
        
        auto db_links = mapper.findBy(criteria);
        links.reserve(db_links.size());
        
        for (const auto& db_link : db_links) {
            links.push_back(utils::DatabaseConverter::convertLink(db_link));
        }
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load links: " + std::string(e.what()));
    }
    
    return links;
}

void RichDatabaseLoader::updateCache(const std::string& map_name, CachedMapData&& data) {
    // Check if we need to evict old entries
    if (cache_.size() >= default_config_.max_cache_size) {
        evictOldestCache();
    }
    
    cache_[map_name] = std::move(data);
}

std::optional<CachedMapData> RichDatabaseLoader::getCachedData(const std::string& map_name) const {
    auto it = cache_.find(map_name);
    if (it == cache_.end()) {
        return std::nullopt;
    }
    
    return it->second;
}

void RichDatabaseLoader::evictOldestCache() {
    if (cache_.empty()) return;
    
    // Find the entry with the oldest last_accessed time
    auto oldest_it = std::min_element(cache_.begin(), cache_.end(),
        [](const auto& a, const auto& b) {
            return a.second.last_accessed < b.second.last_accessed;
        });
    
    if (oldest_it != cache_.end()) {
        // If this is the current map, reset pointers
        if (current_map_name_ == oldest_it->first) {
            current_nodes_ = nullptr;
            current_links_ = nullptr;
            current_adjacency_list_ = nullptr;
            current_map_name_.clear();
            current_map_id_ = -1;
        }
        
        cache_.erase(oldest_it);
    }
}

bool RichDatabaseLoader::isCacheValid(const CachedMapData& data, const DatabaseLoadConfig& config) const {
    if (!config.enable_cache) {
        return false;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::minutes>(now - data.loaded_at);
    
    return age < config.cache_ttl;
}

void RichDatabaseLoader::buildAdjacencyList(const std::vector<NodeInfo>& nodes,
                                           const std::vector<LinkInfo>& links,
                                           std::unordered_map<int32_t, std::vector<int32_t>>& adjacency_list) {
    // Initialize adjacency list for all nodes
    for (const auto& node : nodes) {
        adjacency_list[node.id] = {};
    }
    
    // Add links to adjacency list
    for (const auto& link : links) {
        if (adjacency_list.find(link.id_start) != adjacency_list.end()) {
            adjacency_list[link.id_start].push_back(link.id_straight_link);
        }
    }
}

void RichDatabaseLoader::applyFilters(std::vector<NodeInfo>& nodes, 
                                     std::vector<LinkInfo>& links,
                                     const DatabaseLoadConfig& config) {
    // Apply bounding box filter
    if (config.bounding_box_min && config.bounding_box_max) {
        double min_x = config.bounding_box_min->first;
        double min_y = config.bounding_box_min->second;
        double max_x = config.bounding_box_max->first;
        double max_y = config.bounding_box_max->second;
        
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
            [min_x, min_y, max_x, max_y](const NodeInfo& node) {
                return node.x < min_x || node.x > max_x || 
                       node.y < min_y || node.y > max_y;
            }), nodes.end());
    }
    
    // Apply node type filter
    if (config.node_types_filter) {
        const auto& allowed_types = *config.node_types_filter;
        
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
            [&allowed_types](const NodeInfo& node) {
                return std::find(allowed_types.begin(), allowed_types.end(), 
                               node.type) == allowed_types.end();
            }), nodes.end());
    }
    
    // Remove links that reference filtered nodes
    std::set<int32_t> valid_node_ids;
    for (const auto& node : nodes) {
        valid_node_ids.insert(node.id);
    }
    
    links.erase(std::remove_if(links.begin(), links.end(),
        [&valid_node_ids](const LinkInfo& link) {
            return valid_node_ids.find(link.id_start) == valid_node_ids.end() ||
                   valid_node_ids.find(link.id_end) == valid_node_ids.end();
        }), links.end());
}

} // namespace core
} // namespace vrobot_route_follow