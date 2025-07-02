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

  // Hash function for EdgePair
  struct EdgePairHash {
    std::size_t operator()(const EdgePair &pair) const {
      return std::hash<NodeID>()(pair.first) ^
             (std::hash<NodeID>()(pair.second) << 1);
    }
  };

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
   * @brief Construct graph from nodes and edges with velocity
   * @param nodes Map of nodeID to pose
   * @param edges Vector of (fromID, toID, weight, velocity) tuples
   */
  template <typename VelocityType>
  GraphBase(
      const std::unordered_map<NodeID, Pose2D> &nodes,
      const std::vector<std::tuple<NodeID, NodeID, WeightType, VelocityType>>
          &edges) {
    buildGraphWithVelocity(nodes, edges);
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

  /**
   * @brief Build graph from nodes and edges with velocity data
   * @param nodes Map of nodeID to pose
   * @param edges Vector of (fromID, toID, weight, velocity) tuples
   */
  template <typename VelocityType>
  void buildGraphWithVelocity(
      const std::unordered_map<NodeID, Pose2D> &nodes,
      const std::vector<std::tuple<NodeID, NodeID, WeightType, VelocityType>>
          &edges) {
    nodePoses_ = nodes;

    // Initialize adjacency list and velocity map
    for (const auto &np : nodes) {
      adjList_[np.first] = {};
    }

    // Add directed edges with velocity
    for (const auto &e : edges) {
      NodeID       u, v;
      WeightType   w;
      VelocityType vel;
      std::tie(u, v, w, vel) = e;
      adjList_[u].emplace_back(v, w);

      // Store velocity for this edge
      edgeVelocities_[{u, v}] = static_cast<double>(vel);
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
    edgeVelocities_.clear();
  }

  /**
   * @brief Get velocity for an edge
   * @param fromNode Source node
   * @param toNode Target node
   * @return Velocity for the edge, or 1.0 if not found
   */
  double getEdgeVelocity(const NodeID &fromNode, const NodeID &toNode) const {
    auto it = edgeVelocities_.find({fromNode, toNode});
    if (it != edgeVelocities_.end()) {
      return it->second;
    }
    return 1.0; // Default velocity if not found
  }

  /**
   * @brief Check if edge has velocity data
   * @param fromNode Source node
   * @param toNode Target node
   * @return True if edge has velocity data
   */
  bool hasEdgeVelocity(const NodeID &fromNode, const NodeID &toNode) const {
    return edgeVelocities_.find({fromNode, toNode}) != edgeVelocities_.end();
  }

protected:
  // ========================================================================
  // PROTECTED MEMBER VARIABLES
  // ========================================================================

  std::unordered_map<NodeID, Pose2D> nodePoses_;
  std::unordered_map<NodeID, std::vector<std::pair<NodeID, WeightType>>>
                                                     adjList_;
  std::unordered_map<EdgePair, double, EdgePairHash> edgeVelocities_;
};

} // namespace core
} // namespace vrobot_route_follow