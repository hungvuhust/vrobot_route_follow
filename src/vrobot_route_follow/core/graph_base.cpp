#include "vrobot_route_follow/core/graph_base.hpp"

namespace vrobot_route_follow {
namespace core {

// Explicit template instantiations for common types
template class GraphBase<int, CPose2D, double>;
template class GraphBase<size_t, CPose2D, double>;
template class GraphBase<long, CPose2D, double>;

// Implementation for commonly used methods can be added here
// Currently, most methods are inline in the header

} // namespace core
} // namespace vrobot_route_follow