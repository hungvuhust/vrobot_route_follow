#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

#include <drogon/orm/DbClient.h>
#include <Eigen/Dense>

#include "vrobot_route_follow/data_structures/node_info.hpp"
#include "vrobot_route_follow/data_structures/link_info.hpp"
#include "vrobot_route_follow/data_structures/curve_link_info.hpp"

using vrobot_route_follow::data_structures::NodeInfo;
using vrobot_route_follow::data_structures::LinkInfo;
using vrobot_route_follow::data_structures::CurveLinkInfo;

namespace vrobot_route_follow {
namespace core {

/**
 * @brief Statistics about loaded data
 */
struct LoadStatistics {
    size_t nodes_loaded = 0;
    size_t links_loaded = 0;
    size_t curved_links_loaded = 0;
    std::chrono::milliseconds load_time{0};
    bool cache_hit = false;
    std::string map_name;
    int32_t map_id = -1;
};

/**
 * @brief Configuration for database loading
 */
struct DatabaseLoadConfig {
    // Cache settings
    bool enable_cache = true;
    std::chrono::minutes cache_ttl{30}; // Time to live for cache
    size_t max_cache_size = 10; // Maximum number of maps in cache
    
    // Loading constraints
    std::optional<std::pair<double, double>> bounding_box_min;
    std::optional<std::pair<double, double>> bounding_box_max;
    std::optional<std::vector<std::string>> node_types_filter;
    std::optional<std::vector<std::string>> link_types_filter;
    
    // Performance settings
    size_t batch_size = 1000; // Batch size for loading large datasets
    bool parallel_loading = true;
    std::chrono::seconds connection_timeout{30};
};

/**
 * @brief Cached map data
 */
struct CachedMapData {
    std::unordered_map<int32_t, NodeInfo> nodes;
    std::unordered_map<int32_t, LinkInfo> links;
    std::unordered_map<int32_t, CurveLinkInfo> curved_links;
    std::unordered_map<int32_t, std::vector<int32_t>> adjacency_list;
    std::chrono::steady_clock::time_point last_accessed;
    std::chrono::steady_clock::time_point loaded_at;
    LoadStatistics load_stats;
};

/**
 * @brief Database loader and cache manager for RichGraph
 * 
 * This class handles:
 * - Loading map data from database using Drogon ORM
 * - Intelligent caching with TTL and LRU eviction
 * - Data filtering and batch loading
 * - Connection management and error recovery
 * - Performance monitoring and statistics
 */
class RichDatabaseLoader {
public:
    // Constructor and destructor
    explicit RichDatabaseLoader(std::shared_ptr<drogon::orm::DbClient> db_client);
    ~RichDatabaseLoader() = default;

    // Copy/move semantics
    RichDatabaseLoader(const RichDatabaseLoader&) = delete;
    RichDatabaseLoader& operator=(const RichDatabaseLoader&) = delete;
    RichDatabaseLoader(RichDatabaseLoader&&) = default;
    RichDatabaseLoader& operator=(RichDatabaseLoader&&) = default;

    // ========================================================================
    // PRIMARY LOADING INTERFACE
    // ========================================================================

    /**
     * @brief Load map data with automatic caching
     * @param map_name Name of the map to load
     * @param config Loading configuration (optional)
     * @return Load statistics and success status
     */
    LoadStatistics loadMap(const std::string& map_name, 
                          const DatabaseLoadConfig& config = DatabaseLoadConfig{});

    /**
     * @brief Get loaded nodes for current map
     * @return Reference to nodes map
     */
    const std::unordered_map<int32_t, NodeInfo>& getNodes() const;

    /**
     * @brief Get loaded links for current map  
     * @return Reference to links map
     */
    const std::unordered_map<int32_t, LinkInfo>& getLinks() const;

    /**
     * @brief Get loaded curved links for current map
     * @return Reference to curved links map
     */
    const std::unordered_map<int32_t, CurveLinkInfo>& getCurvedLinks() const;

    /**
     * @brief Get adjacency list for current map
     * @return Reference to adjacency list
     */
    const std::unordered_map<int32_t, std::vector<int32_t>>& getAdjacencyList() const;

    // ========================================================================
    // CACHE MANAGEMENT
    // ========================================================================

    /**
     * @brief Check if map is in cache
     * @param map_name Map name to check
     * @return True if map is cached and valid
     */
    bool isMapCached(const std::string& map_name) const;

    /**
     * @brief Force reload map from database (bypass cache)
     * @param map_name Map name to reload
     * @param config Loading configuration
     * @return Load statistics
     */
    LoadStatistics reloadMap(const std::string& map_name,
                            const DatabaseLoadConfig& config = DatabaseLoadConfig{});

    /**
     * @brief Clear cache for specific map
     * @param map_name Map name to remove from cache
     */
    void clearMapCache(const std::string& map_name);

    /**
     * @brief Clear all cached data
     */
    void clearAllCache();

    /**
     * @brief Get cache statistics
     * @return Cache hit rate, size, etc.
     */
    std::unordered_map<std::string, double> getCacheStatistics() const;

    // ========================================================================
    // FILTERED LOADING
    // ========================================================================

    /**
     * @brief Load only nodes within bounding box
     * @param map_name Map name
     * @param min_coords Minimum coordinates (x, y)
     * @param max_coords Maximum coordinates (x, y)
     * @return Load statistics
     */
    LoadStatistics loadMapInBounds(const std::string& map_name,
                                  const std::pair<double, double>& min_coords,
                                  const std::pair<double, double>& max_coords);

    /**
     * @brief Load only specific node types
     * @param map_name Map name
     * @param node_types List of node types to load
     * @return Load statistics
     */
    LoadStatistics loadMapByNodeTypes(const std::string& map_name,
                                     const std::vector<std::string>& node_types);

    // ========================================================================
    // STATUS AND DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Get current map name
     * @return Current map name (empty if none loaded)
     */
    const std::string& getCurrentMapName() const { return current_map_name_; }

    /**
     * @brief Get current map ID
     * @return Current map ID (-1 if none loaded)
     */
    int32_t getCurrentMapId() const { return current_map_id_; }

    /**
     * @brief Check if database connection is valid
     * @return True if connection is healthy
     */
    bool isDatabaseConnected() const;

    /**
     * @brief Get last load statistics
     * @return Statistics from last successful load
     */
    const LoadStatistics& getLastLoadStats() const { return last_load_stats_; }

    /**
     * @brief Validate loaded data integrity
     * @return Validation results
     */
    std::vector<std::string> validateDataIntegrity() const;

private:
    // ========================================================================
    // INTERNAL IMPLEMENTATION
    // ========================================================================

    // Database operations
    LoadStatistics loadFromDatabase(const std::string& map_name, 
                                   const DatabaseLoadConfig& config);
    std::optional<int32_t> getMapId(const std::string& map_name);
    std::vector<NodeInfo> loadNodes(int32_t map_id, const DatabaseLoadConfig& config);
    std::vector<LinkInfo> loadLinks(int32_t map_id, const DatabaseLoadConfig& config);

    // Cache operations
    void updateCache(const std::string& map_name, CachedMapData&& data);
    std::optional<CachedMapData> getCachedData(const std::string& map_name) const;
    void evictOldestCache();
    bool isCacheValid(const CachedMapData& data, const DatabaseLoadConfig& config) const;

    // Data processing
    void buildAdjacencyList(const std::vector<NodeInfo>& nodes,
                           const std::vector<LinkInfo>& links,
                           std::unordered_map<int32_t, std::vector<int32_t>>& adjacency_list);
    void applyFilters(std::vector<NodeInfo>& nodes, 
                     std::vector<LinkInfo>& links,
                     const DatabaseLoadConfig& config);

    // Member variables
    std::shared_ptr<drogon::orm::DbClient> db_client_;
    
    // Current state
    std::string current_map_name_;
    int32_t current_map_id_ = -1;
    
    // Cache storage
    mutable std::unordered_map<std::string, CachedMapData> cache_;
    DatabaseLoadConfig default_config_;
    
    // Statistics
    LoadStatistics last_load_stats_;
    mutable size_t cache_hits_ = 0;
    mutable size_t cache_misses_ = 0;
    
    // Current data (references to cached data)
    const std::unordered_map<int32_t, NodeInfo>* current_nodes_ = nullptr;
    const std::unordered_map<int32_t, LinkInfo>* current_links_ = nullptr;
    const std::unordered_map<int32_t, CurveLinkInfo>* current_curved_links_ = nullptr;
    const std::unordered_map<int32_t, std::vector<int32_t>>* current_adjacency_list_ = nullptr;
};

} // namespace core
} // namespace vrobot_route_follow