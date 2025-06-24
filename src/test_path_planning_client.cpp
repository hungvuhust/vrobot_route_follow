#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vrobot_route_follow/srv/path_planning.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

class PathPlanningClient : public rclcpp::Node {
public:
  PathPlanningClient() : Node("path_planning_client") {
    client_ =
        this->create_client<vrobot_route_follow::srv::PathPlanning>("plan_path");

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/route_path", rclcpp::QoS(10).transient_local().reliable());

    RCLCPP_INFO(this->get_logger(), "Path Planning Client started");
  }

  void callService(std::string map_name, uint64_t target_node_id, double x,
                   double y, double theta) {
    // Đợi service sẵn sàng
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Client interrupted");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
    }

    // Tạo request
    auto request =
        std::make_shared<vrobot_route_follow::srv::PathPlanning::Request>();
    request->target_node_id     = target_node_id;
    request->current_pose.x     = x;
    request->current_pose.y     = y;
    request->current_pose.theta = theta;
    request->map_name           = map_name;
    RCLCPP_INFO(this->get_logger(),
                "Sending request: target_node=%lu, pose=(%f,%f,%f)",
                target_node_id, x, y, theta);

    // Gửi request
    auto future = client_->async_send_request(request);

    // Đợi response
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {

      auto response = future.get();

      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✅ Path planning successfully!");
        RCLCPP_INFO(this->get_logger(), "   Algorithm: %s",
                    response->algorithm_used.c_str());
        RCLCPP_INFO(this->get_logger(), "   Distance: %f",
                    response->total_distance);
        RCLCPP_INFO(this->get_logger(), "   Path poses: %zu",
                    response->path.poses.size());

        // Pub
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp    = this->now();
        path.poses           = response->path.poses;
        path_publisher_->publish(path);

        // In ra một số poses đầu và cuối
        if (!response->path.poses.empty()) {
          const auto &first_pose = response->path.poses.front();
          const auto &last_pose  = response->path.poses.back();

          RCLCPP_INFO(this->get_logger(), "   First pose: (%f,%f,%f)",
                      first_pose.pose.position.x, first_pose.pose.position.y,
                      first_pose.pose.orientation.z);
          RCLCPP_INFO(this->get_logger(), "   Last pose: (%f,%f,%f)",
                      last_pose.pose.position.x, last_pose.pose.position.y,
                      last_pose.pose.orientation.z);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Path planning failed!");
        RCLCPP_ERROR(this->get_logger(), "   Algorithm: %s",
                     response->algorithm_used.c_str());
        RCLCPP_ERROR(this->get_logger(), "   Error: %s",
                     response->error_message.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error calling service");
    }
  }

private:
  rclcpp::Client<vrobot_route_follow::srv::PathPlanning>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<PathPlanningClient>();

  // Test case 1: Target node gần
  RCLCPP_INFO(client_node->get_logger(), "\n=== Test 1: Target node near ===");
  client_node->callService("mapmoi", 1, 0.0, 0.0, 0.0);

  std::this_thread::sleep_for(10s);

  // Test case 2: Target node xa
  RCLCPP_INFO(client_node->get_logger(), "\n=== Test 2: Target node far ===");
  client_node->callService("mapmoi", 3, 2.5, -0.5, 0.0);

  std::this_thread::sleep_for(10s);

  // Test case 3: Target node không tồn tại
  RCLCPP_INFO(client_node->get_logger(),
              "\n=== Test 3: Target node not found ===");
  client_node->callService("mapmoi", 9999, 1.0, 1.0, 0.0);

  rclcpp::shutdown();
  return 0;
}