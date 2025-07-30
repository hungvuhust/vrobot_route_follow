#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// Database
#include <drogon/HttpAppFramework.h>
#include <drogon/orm/DbClient.h>

// New modular architecture
#include "vrobot_route_follow/core/rich_database_loader.hpp"
#include "vrobot_route_follow/core/rich_path_planner.hpp"
#include "vrobot_route_follow/core/rich_path_optimizer.hpp"
#include "vrobot_route_follow/core/rich_path_validator.hpp"
#include "vrobot_route_follow/utils/database_converter.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"

// ROS messages và services
#include "vrobot_route_follow/srv/path_planning.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>

using namespace drogon;
using namespace drogon::orm;
using namespace vrobot_route_follow::core;
using namespace vrobot_route_follow::utils;

class PathPlanningService : public rclcpp::Node {
private:
  rclcpp::Service<vrobot_route_follow::srv::PathPlanning>::SharedPtr service_;
  std::shared_ptr<drogon::orm::DbClient>                             db_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;

  // New modular architecture components
  std::shared_ptr<RichDatabaseLoader> database_loader_;
  std::shared_ptr<RichPathPlanner>    path_planner_;
  std::shared_ptr<RichPathOptimizer>  path_optimizer_;
  std::shared_ptr<RichPathValidator>  path_validator_;
  
  // Current loaded map
  std::string current_map_name_;
  bool        map_loaded_;

public:
  PathPlanningService() : Node("path_planning_service"), map_loaded_(false) {
    // Khởi tạo database client
    initDatabase();

    // Khởi tạo modular architecture components
    database_loader_ = std::make_shared<RichDatabaseLoader>(db_client_);
    path_planner_ = std::make_shared<RichPathPlanner>(database_loader_);
    path_optimizer_ = std::make_shared<RichPathOptimizer>();
    path_validator_ = std::make_shared<RichPathValidator>();

    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph_vis", rclcpp::QoS(10).transient_local().reliable());

    // Tạo service
    service_ = this->create_service<vrobot_route_follow::srv::PathPlanning>(
        "plan_path",
        std::bind(&PathPlanningService::handlePathPlanningRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Path Planning Service with Modular Architecture started");
  }

  ~PathPlanningService() {
    RCLCPP_INFO(this->get_logger(), "Path Planning Service stopped");
  }

private:
  void initDatabase() {
    db_client_ = DbClient::newPgClient(
        "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345",
        1);

    if (!db_client_) {
      RCLCPP_ERROR(this->get_logger(), "Cannot connect to database");
      throw std::runtime_error("Database connection failed");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to database successfully");
  }

  bool loadMapData(const std::string& map_name) {
    try {
      // Use RichDatabaseLoader to load map data
      auto load_stats = database_loader_->loadMap(map_name);
      
      if (load_stats.nodes_loaded == 0 || load_stats.links_loaded == 0) {
        throw std::runtime_error("No data found for map: " + map_name);
      }

      current_map_name_ = map_name;
      map_loaded_ = true;
      
      RCLCPP_INFO(this->get_logger(),
                  "Loaded map '%s' successfully: %zu nodes, %zu links (cache hit: %s)",
                  map_name.c_str(), load_stats.nodes_loaded, load_stats.links_loaded,
                  load_stats.cache_hit ? "true" : "false");

      // TODO: Add visualization using RichGraph data
      // auto vis_markers = createVisualizationMarkers();
      // pub_viz_->publish(vis_markers);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading map '%s': %s", 
                   map_name.c_str(), e.what());
      map_loaded_ = false;
      return false;
    }

    return true;
  }

  void handlePathPlanningRequest(
      const std::shared_ptr<vrobot_route_follow::srv::PathPlanning::Request>
          request,
      std::shared_ptr<vrobot_route_follow::srv::PathPlanning::Response>
          response) {

    RCLCPP_INFO(this->get_logger(),
                "Received path planning request: target_node=%lu, "
                "current_pose=(%f,%f,%f) of map %s",
                request->target_node_id, request->current_pose.x,
                request->current_pose.y, request->current_pose.theta,
                request->map_name.c_str());

    try {
      // Load map data if needed
      if (current_map_name_ != request->map_name || !map_loaded_) {
        if (!loadMapData(request->map_name)) {
          response->success = false;
          response->error_message = "Failed to load map data";
          response->algorithm_used = "none";
          return;
        }
      }

      // Convert current pose to Eigen format
      Eigen::Vector3d current_pose(request->current_pose.x, 
                                   request->current_pose.y, 
                                   request->current_pose.theta);

      // Create planning request using new modular architecture
      PlanningRequest planning_request;
      planning_request.start_pose = current_pose;
      planning_request.goal_node_id = request->target_node_id;
      planning_request.request_id = "service_request_" + std::to_string(this->now().nanoseconds());
      planning_request.timestamp = std::chrono::steady_clock::now();

      // Execute path planning using RichPathPlanner
      auto rich_result = path_planner_->planPath(planning_request);

      // Validate the result
      auto validation_result = path_validator_->validatePath(rich_result);
      if (!validation_result.is_valid) {
        RCLCPP_WARN(this->get_logger(), "Path validation failed with %zu issues", 
                    validation_result.issues.size());
        for (const auto& issue : validation_result.issues) {
          if (issue.severity == ValidationSeverity::ERROR) {
            response->success = false;
            response->error_message = "Path validation failed: " + issue.description;
            response->algorithm_used = rich_result.algorithmUsed;
            return;
          }
        }
      }

      // Optimize the path (optional)
      if (rich_result.success) {
        rich_result = path_optimizer_->quickOptimize(rich_result, OptimizationObjective::BALANCED);
      }

      // Prepare response
      response->success = rich_result.success;
      response->algorithm_used = rich_result.algorithmUsed;
      response->error_message = rich_result.errorMessage;
      response->total_distance = rich_result.totalDistance;

      if (rich_result.success) {
        // Convert RichPathResult to nav_msgs::Path
        response->path = vrobot_route_follow::utils::DatabaseConverter::richPathToNavPath(
            rich_result, "map", this->now());

        RCLCPP_INFO(
            this->get_logger(),
            "Path planning successful: algorithm=%s, distance=%.2f, nodes=%zu, poses=%zu",
            rich_result.algorithmUsed.c_str(), rich_result.totalDistance,
            rich_result.nodeSequence.size(), response->path.poses.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "Path planning failed: %s - %s",
                    rich_result.algorithmUsed.c_str(), rich_result.errorMessage.c_str());
      }

    } catch (const std::exception &e) {
      response->success = false;
      response->error_message = std::string("Exception: ") + e.what();
      response->algorithm_used = "exception";
      RCLCPP_ERROR(this->get_logger(), "Exception in path planning: %s", e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto service_node = std::make_shared<PathPlanningService>();

    RCLCPP_INFO(service_node->get_logger(),
                "Path Planning Service is running...");
    rclcpp::spin(service_node);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("path_planning_service"),
                 "Error starting service: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}