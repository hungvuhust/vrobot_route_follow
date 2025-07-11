# TODO - Cải tiến Thuật toán Path Planning

## 1. Vấn đề Hiện tại

### 1.1 Vấn đề chính:
- **Mất dữ liệu**: Hiện tại chỉ lấy tọa độ (x, y, theta) từ Node và weight từ Link, bỏ qua các thuộc tính quan trọng khác
- **Khó trace**: Không thể trace ngược lại để biết robot đi qua node/link nào, chỉ có PathSegment (pose → pose)
- **Khó mở rộng**: Khi cần thêm thuộc tính mới (max_velocity, type, node_name, ...) phải sửa đổi nhiều nơi
- **Template phức tạp**: Sử dụng template generic làm code khó hiểu và maintain

### 1.2 Ví dụ cụ thể:
```cpp
// Hiện tại: Chỉ có tọa độ
std::vector<PathSegment> pathSegments; // PathSegment = std::pair<Pose2D, Pose2D>

// Cần: Biết đi qua node/link nào
std::vector<NodeInfo> nodeSequence;
std::vector<LinkInfo> linkSequence;
```

## 2. Ý tưởng Cải tiến

### 2.1 Cấu trúc dữ liệu mới

#### 2.1.1 Node Structure
```cpp
struct NodeInfo {
    int32_t id;
    std::string node_name;
    float x, y, theta;
    std::string type;
    int32_t map_id;
    
    // Thêm các thuộc tính khác nếu cần
    std::optional<double> max_waiting_time;
    std::optional<std::string> zone_type;
    
    CPose2D toPose() const { return CPose2D(x, y, theta); }
};
```

#### 2.1.2 Link Structure
```cpp
struct LinkInfo {
    int32_t id_straight_link;
    int32_t id_start;
    int32_t id_end;
    int32_t map_id;
    double max_velocity;
    
    // Thêm các thuộc tính khác nếu cần
    std::optional<std::string> link_type;
    std::optional<double> width;
    std::optional<bool> bidirectional;
    
    double calculateWeight(const NodeInfo& start, const NodeInfo& end) const {
        CPose2D startPose = start.toPose();
        CPose2D endPose = end.toPose();
        return (endPose - startPose).norm();
    }
};
```

### 2.2 Path Result mới

#### 2.2.1 Rich Path Result
```cpp
struct RichPathResult {
    std::vector<NodeInfo> nodeSequence;      // Sequence of nodes
    std::vector<LinkInfo> linkSequence;      // Sequence of links
    std::vector<CPose2D> poseSequence;       // Interpolated poses
    double totalDistance;
    std::string algorithmUsed;
    bool success;
    
    // Conversion methods
    std::vector<PathSegment> toPathSegments() const;
    nav_msgs::msg::Path toNavPath(const std::string& frame_id, const rclcpp::Time& timestamp) const;
    vrobot_local_planner::msg::Path toVPath(const std::string& frame_id, const rclcpp::Time& timestamp) const;
};
```

### 2.3 Graph Structure mới

#### 2.3.1 Bỏ template, sử dụng struct cụ thể
```cpp
class RichGraph {
private:
    std::unordered_map<int32_t, NodeInfo> nodes_;
    std::unordered_map<int32_t, LinkInfo> links_;
    std::unordered_map<int32_t, std::vector<int32_t>> adjacency_list_; // node_id -> list of link_ids
    
public:
    // Load from database
    bool loadFromDatabase(const std::string& map_name, std::shared_ptr<drogon::orm::DbClient> db_client);
    
    // Path planning
    RichPathResult planPath(const CPose2D& start_pose, int32_t target_node_id, const PlanningConfig& config);
    
    // Accessors
    const NodeInfo& getNode(int32_t node_id) const;
    const LinkInfo& getLink(int32_t link_id) const;
    std::vector<int32_t> getNodeNeighborLinks(int32_t node_id) const;
    
    // Utilities
    int32_t findClosestNode(const CPose2D& pose) const;
    std::vector<int32_t> findClosestLinks(const CPose2D& pose, size_t max_links) const;
};
```

## 3. Kế hoạch Thực hiện

### 3.1 Phase 1: Tạo cấu trúc dữ liệu mới (Tuần 1)

#### 3.1.1 Tạo data structures
- [x] Tạo `include/vrobot_route_follow/data_structures/node_info.hpp`
- [x] Tạo `include/vrobot_route_follow/data_structures/link_info.hpp`
- [x] Tạo `include/vrobot_route_follow/data_structures/rich_path_result.hpp`

#### 3.1.2 Tạo conversion utilities
- [x] Tạo `include/vrobot_route_follow/utils/database_converter.hpp`
- [ ] Implement conversion từ ORM objects sang struct mới (implementation files)
- [ ] Implement conversion từ struct sang ROS messages (implementation files)

### 3.2 Phase 2: Tạo RichGraph class (Tuần 2)

#### 3.2.1 Core graph functionality
- [ ] Tạo `include/vrobot_route_follow/core/rich_graph.hpp`
- [ ] Implement database loading
- [ ] Implement basic graph operations (adjacency, neighbors, etc.)

#### 3.2.2 Geometric utilities
- [ ] Tạo `include/vrobot_route_follow/core/rich_geometric_utils.hpp`
- [ ] Implement closest node/link finding
- [ ] Implement projection onto links

### 3.3 Phase 3: Implement path planning algorithms (Tuần 3-4)

#### 3.3.1 Basic algorithms
- [ ] Tạo `include/vrobot_route_follow/algorithms/rich_pathfinding.hpp`
- [ ] Implement Dijkstra với RichPathResult
- [ ] Implement direct path planning

#### 3.3.2 Advanced algorithms
- [ ] Tạo `include/vrobot_route_follow/algorithms/rich_link_based_planner.hpp`
- [ ] Implement modular link approach với rich data
- [ ] Implement scoring system với rich attributes

### 3.4 Phase 4: Tạo modular architecture (Tuần 5)

#### 3.4.1 Tách thành modules
- [ ] `RichDatabaseLoader`: Load và cache dữ liệu từ database
- [ ] `RichPathPlanner`: Core path planning logic
- [ ] `RichPathOptimizer`: Tối ưu hóa path
- [ ] `RichPathValidator`: Validate path trước khi return

#### 3.4.2 Plugin architecture
- [ ] Tạo interface cho các algorithm plugins
- [ ] Implement factory pattern cho algorithm selection
- [ ] Cho phép register custom algorithms

### 3.5 Phase 5: Integration và testing (Tuần 6)

#### 3.5.1 Update services
- [ ] Update `PathPlanningService` để sử dụng RichGraph
- [ ] Update `MoveToPoseActionServer` để sử dụng RichPathResult
- [ ] Maintain backward compatibility

#### 3.5.2 Testing
- [ ] Tạo unit tests cho RichGraph
- [ ] Tạo integration tests
- [ ] Performance comparison với version cũ

## 4. Lợi ích Dự kiến

### 4.1 Traceability
- Biết chính xác robot đi qua node nào, link nào
- Có thể log chi tiết journey của robot
- Dễ debug khi có vấn đề

### 4.2 Extensibility
- Thêm thuộc tính mới không cần sửa core algorithm
- Dễ implement custom scoring functions
- Hỗ trợ nhiều loại constraints (velocity, width, zone restrictions)

### 4.3 Performance
- Cache database queries hiệu quả hơn
- Reduce memory footprint với struct thay vì template
- Faster compilation time

### 4.4 Maintainability
- Code rõ ràng, dễ hiểu hơn
- Modular architecture dễ maintain
- Better separation of concerns

## 5. Rủi ro và Giải pháp

### 5.1 Rủi ro
- **Breaking changes**: API mới có thể break existing code
- **Performance regression**: Struct có thể chậm hơn template
- **Memory usage**: Rich data structures có thể tốn memory hơn

### 5.2 Giải pháp
- **Backward compatibility**: Maintain old API trong giai đoạn transition
- **Performance testing**: Benchmark thoroughly trước khi deploy
- **Memory optimization**: Sử dụng optional fields và smart pointers

## 6. Milestones

- **Milestone 1** (End of Week 2): Basic RichGraph working
- **Milestone 2** (End of Week 4): All algorithms ported
- **Milestone 3** (End of Week 6): Full integration complete
- **Milestone 4** (End of Week 8): Production ready with tests

## 7. Cập nhật quan trọng

### 7.1 Chuyển đổi từ MRPT sang Eigen
- **Quyết định**: Sử dụng Eigen thay vì MRPT cho tính toán hình học
- **Lý do**: Eigen nhẹ hơn, phổ biến hơn, và tích hợp tốt hơn với ROS2
- **Thay đổi**: 
  - `mrpt::poses::CPose2D` → `Eigen::Vector3d` (x, y, theta)
  - `Eigen::Vector2d` cho position (x, y)
  - Tự implement các utility functions cần thiết

### 7.2 Hoàn thành Phase 1
- ✅ **NodeInfo struct**: Chứa tất cả thuộc tính từ database + extended attributes
- ✅ **LinkInfo struct**: Chứa tất cả thuộc tính từ database + extended attributes  
- ✅ **RichPathResult struct**: Kết quả path planning với full traceability
- ✅ **DatabaseConverter class**: Utilities để convert giữa ORM và structs

## 8. Next Steps

1. **Immediate**: Implement các method trong DatabaseConverter
2. **This week**: Tạo implementation files (.cpp) cho các structs
3. **Next week**: Implement basic RichGraph với database loading
4. **Following week**: Port một algorithm đơn giản (Dijkstra) sang rich version 