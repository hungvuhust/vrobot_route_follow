#pragma once

#include <mrpt/poses/CPose2D.h>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace mrpt::poses;

namespace vrobot_route_follow {
namespace core {

/**
 * @brief Base template class for graph structure and basic operations
 * @tparam NodeID Type for node identifiers
 * @tparam Pose2D Type for 2D poses (default: CPose2D)
 * @tparam WeightType Type for edge weights (default: double)
 */
template <typename NodeID, typename Pose2D = CPose2D,
          typename WeightType = double>
class GraphBase {
public:
  using EdgePair = std::pair<NodeID, NodeID>;

  // ========================================================================
  // CONSTRUCTORS & INITIALIZATION
  // ========================================================================

  GraphBase()                             = default;
  GraphBase(const GraphBase &)            = default;
  GraphBase &operator=(const GraphBase &) = default;
  virtual ~GraphBase()                    = default;

  /**
   * @brief Construct graph from nodes and edges
   * @param nodes Map of nodeID to pose
   * @param edges Vector of (fromID, toID, weight) tuples
   */
  GraphBase(const std::unordered_map<NodeID, Pose2D>                  &nodes,
            const std::vector<std::tuple<NodeID, NodeID, WeightType>> &edges) {
    buildGraph(nodes, edges);
  }

  /**
   * @brief Build graph from nodes and edges
   * @param nodes Map of nodeID to pose
   * @param edges Vector of (fromID, toID, weight) tuples
   */
  void
  buildGraph(const std::unordered_map<NodeID, Pose2D>                  &nodes,
             const std::vector<std::tuple<NodeID, NodeID, WeightType>> &edges) {
    nodePoses_ = nodes;

    // Initialize adjacency list
    for (const auto &np : nodes) {
      adjList_[np.first] = {};
    }

    // Add directed edges
    for (const auto &e : edges) {
      NodeID     u, v;
      WeightType w;
      std::tie(u, v, w) = e;
      adjList_[u].emplace_back(v, w);
    }
  }

  // ========================================================================
  // BASIC ACCESSORS
  // ========================================================================

  /**
   * @brief Get pose of a node
   * @param nodeId Node identifier
   * @return Pose of the node
   */
  const Pose2D &getNodePose(const NodeID &nodeId) const {
    auto it = nodePoses_.find(nodeId);
    if (it == nodePoses_.end()) {
      throw std::runtime_error("Node not found: " + std::to_string(nodeId));
    }
    return it->second;
  }

  /**
   * @brief Check if node exists in graph
   * @param nodeId Node identifier
   * @return True if node exists
   */
  bool hasNode(const NodeID &nodeId) const {
    return nodePoses_.find(nodeId) != nodePoses_.end();
  }

  /**
   * @brief Get all node IDs
   * @return Vector of all node IDs
   */
  std::vector<NodeID> getAllNodeIds() const {
    std::vector<NodeID> ids;
    ids.reserve(nodePoses_.size());
    for (const auto &kv : nodePoses_) {
      ids.push_back(kv.first);
    }
    return ids;
  }

  /**
   * @brief Get neighbors of a node
   * @param nodeId Node identifier
   * @return Vector of (neighbor_id, weight) pairs
   */
  const std::vector<std::pair<NodeID, WeightType>> &
  getNeighbors(const NodeID &nodeId) const {
    auto it = adjList_.find(nodeId);
    if (it == adjList_.end()) {
      static const std::vector<std::pair<NodeID, WeightType>> empty;
      return empty;
    }
    return it->second;
  }

  /**
   * @brief Get number of nodes
   * @return Number of nodes in graph
   */
  size_t getNodeCount() const { return nodePoses_.size(); }

  /**
   * @brief Get number of edges
   * @return Total number of directed edges
   */
  size_t getEdgeCount() const {
    size_t count = 0;
    for (const auto &kv : adjList_) {
      count += kv.second.size();
    }
    return count;
  }

  /**
   * @brief Check if graph is empty
   * @return True if graph has no nodes
   */
  bool empty() const { return nodePoses_.empty(); }

  /**
   * @brief Clear all data
   */
  void clear() {
    nodePoses_.clear();
    adjList_.clear();
  }

protected:
  // ========================================================================
  // PROTECTED MEMBER VARIABLES
  // ========================================================================

  std::unordered_map<NodeID, Pose2D> nodePoses_;
  std::unordered_map<NodeID, std::vector<std::pair<NodeID, WeightType>>>
      adjList_;
};

} // namespace core
} // namespace vrobot_route_follow