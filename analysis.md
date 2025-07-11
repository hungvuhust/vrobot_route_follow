# Phân tích Thuật toán và Hướng Giải quyết - vrobot_route_follow

## 1. Tổng quan

Project `vrobot_route_follow` là một package ROS2 được thiết kế để thực hiện việc lập kế hoạch đường đi (path planning) cho robot di động. Package này sử dụng cách tiếp cận dựa trên đồ thị (graph-based) kết hợp với các thuật toán tìm đường đi tối ưu.

### 1.1 Kiến trúc Tổng thể

- **Core Components**: Các thành phần cốt lõi cho việc xử lý đồ thị và hình học
- **Algorithms**: Các thuật toán lập kế hoạch đường đi
- **Utils**: Các tiện ích hỗ trợ như chuyển đổi định dạng và visualization
- **Services & Actions**: Giao diện ROS2 cho việc lập kế hoạch đường đi

## 2. Các Thuật toán Chính

### 2.1 Thuật toán Dijkstra Cơ bản

```cpp
dijkstra(startNode, targetNode)
```

- Thuật toán tìm đường đi ngắn nhất truyền thống
- Sử dụng priority queue để tối ưu hóa việc tìm kiếm
- Trả về cặp (path_segments, total_distance)

### 2.2 Link-based Planning

Package sử dụng 4 phương pháp tiếp cận dựa trên link:

1. **Modular Link Approach**:
```cpp
dijkstraWithModularLinkApproach(startPose, targetNode, directThreshold, maxLinks, linkDistanceWeight, maxLinkDistance, graphDistanceWeight)
```
- Kết hợp nhiều tiêu chí để đánh giá đường đi
- Sử dụng hệ thống scoring để cân bằng giữa khoảng cách tới link và khoảng cách trên đồ thị

2. **Enhanced Link Access**:
```cpp
dijkstraWithLinkAccess(startPose, targetNode, distanceThreshold, maxLinks, linkDistanceWeight)
```
- Phù hợp cho các pose ở xa
- Tối ưu hóa việc tiếp cận các link

3. **Simple Link Access**:
```cpp
dijkstraWithSimpleLinkAccess(startPose, targetNode, maxLinks)
```
- Đơn giản hóa quá trình: start → điểm gần nhất trên link → đích
- Phù hợp cho các trường hợp đơn giản

4. **Link Following**:
```cpp
dijkstraWithLinkFollowing(startPose, targetNode, maxLinks)
```
- Robot đi theo topology của link
- Có phiên bản nâng cao (Smart Link Following) với trọng số thích ứng

### 2.3 Hệ thống Scoring

```cpp
calculateWeightedScore(linkDistance, graphDistance, linkWeight, graphWeight)
```

- Đánh giá chất lượng đường đi dựa trên nhiều tiêu chí
- Cân bằng giữa:
  - Khoảng cách tới link gần nhất
  - Khoảng cách đi qua đồ thị
  - Các trọng số có thể điều chỉnh

## 3. Quy trình Lập kế hoạch Đường đi

1. **Kiểm tra Đường đi Trực tiếp**:
   - Nếu khoảng cách < directThreshold → sử dụng đường thẳng
   
2. **Kiểm tra Dijkstra Chuẩn**:
   - Nếu gần node (distanceToClosest <= distanceThreshold)
   - Hoặc không bật link-based planning
   
3. **Sử dụng Link-based Approach**:
   - Tìm các link gần nhất
   - Đánh giá mỗi link bằng hệ thống scoring
   - Chọn đường đi tốt nhất
   
4. **Fallback về Dijkstra**:
   - Nếu các phương pháp trên thất bại

## 4. Tích hợp với ROS2

### 4.1 Services

- **PathPlanning Service**: 
  - Input: map_name, target_node_id, current_pose
  - Output: path, success, algorithm_used, total_distance

### 4.2 Actions

- **MoveToPose Action**:
  - Thực hiện việc di chuyển robot theo path đã tính toán
  - Chia path thành các segments để thực hiện
  - Theo dõi và phản hồi tiến trình thực hiện

## 5. Lưu trữ và Quản lý Dữ liệu

- Sử dụng PostgreSQL để lưu trữ:
  - Bản đồ (Map)
  - Các node
  - Các link
- Cache dữ liệu để tránh tải lại database thường xuyên

## 6. Visualization

- Hiển thị đồ thị và đường đi trên RViz
- Sử dụng MarkerArray để visualization
- Hỗ trợ theo dõi quá trình thực thi

## 7. Tối ưu hóa và Cải tiến

1. **Pruning**: Có thể bật/tắt để giảm không gian tìm kiếm
2. **Caching**: Lưu trữ tạm thời để tăng hiệu suất
3. **Adaptive Weights**: Trọng số thích ứng trong Smart Link Following
4. **Configurable Parameters**: Nhiều tham số có thể điều chỉnh để phù hợp với từng use case 