#include <cmath>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// Database
#include <drogon/HttpAppFramework.h>
#include <drogon/orm/DbClient.h>
#include <drogon/orm/Mapper.h>

// New modular architecture
#include "vrobot_route_follow/core/rich_database_loader.hpp"
#include "vrobot_route_follow/core/rich_path_planner.hpp"
#include "vrobot_route_follow/core/rich_path_optimizer.hpp"
#include "vrobot_route_follow/core/rich_path_validator.hpp"
#include "vrobot_route_follow/utils/database_converter.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"

// Action messages
#include "vrobot_local_planner/action/follow_path.hpp"
#include "vrobot_local_planner/action/v_follow_path.hpp"
#include "vrobot_route_follow/action/move_to_pose.hpp"

// Geometry messages
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>

using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::amr_01;
using namespace std::placeholders;
using namespace vrobot_route_follow::core;
using namespace vrobot_route_follow::utils;

class MoveToPoseActionServer : public rclcpp::Node {
private:
  rclcpp_action::Server<vrobot_route_follow::action::MoveToPose>::SharedPtr
      action_server_;
  rclcpp_action::Client<vrobot_local_planner::action::FollowPath>::SharedPtr
      follow_path_client_;
  rclcpp_action::Client<vrobot_local_planner::action::VFollowPath>::SharedPtr
      v_follow_path_client_;

  // Database components
  std::shared_ptr<drogon::orm::DbClient> db_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<vrobot_local_planner::msg::Path>::SharedPtr
      vpath_publisher_;

  // New modular architecture components
  std::shared_ptr<RichDatabaseLoader> database_loader_;
  std::shared_ptr<RichPathPlanner>    path_planner_;
  std::shared_ptr<RichPathOptimizer>  path_optimizer_;
  std::shared_ptr<RichPathValidator>  path_validator_;
  
  // Current loaded map
  std::string current_map_name_;
  bool        map_loaded_;

public:
  MoveToPoseActionServer() : Node("move_to_pose_action_server"), map_loaded_(false) {

    // Initialize database client
    initDatabase();

    // Initialize modular architecture components
    database_loader_ = std::make_shared<RichDatabaseLoader>(db_client_);
    path_planner_ = std::make_shared<RichPathPlanner>(database_loader_);
    path_optimizer_ = std::make_shared<RichPathOptimizer>();
    path_validator_ = std::make_shared<RichPathValidator>();

    // Create publisher for visualization
    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph_vis", rclcpp::QoS(10).transient_local().reliable());

    // Create publisher for path
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "route_path", rclcpp::QoS(10).transient_local().reliable());

    vpath_publisher_ = this->create_publisher<vrobot_local_planner::msg::Path>(
        "route_vpath", rclcpp::QoS(10).transient_local().reliable());

    // Create follow path action client
    follow_path_client_ =
        rclcpp_action::create_client<vrobot_local_planner::action::FollowPath>(
            this, "follow_path");

    // Create v follow path action client
    v_follow_path_client_ =
        rclcpp_action::create_client<vrobot_local_planner::action::VFollowPath>(
            this, "v_follow_path");

    // Create action server
    action_server_ =
        rclcpp_action::create_server<vrobot_route_follow::action::MoveToPose>(
            this, "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Move To Pose Action Server started");
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

      // Visualize loaded map using new architecture
      const auto& nodes = database_loader_->getNodes();
      const auto& links = database_loader_->getLinks();
      auto map_markers = RichVisualization::createMapMarkers(
          nodes, links, "map", this->now(), 0.1, 0.05);
      pub_viz_->publish(map_markers);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading map '%s': %s", 
                   map_name.c_str(), e.what());
      map_loaded_ = false;
      return false;
    }

    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const vrobot_route_follow::action::MoveToPose::Goal>
          goal) {

    RCLCPP_INFO(this->get_logger(),
                "Received goal: map=%s, target_node=%lu, target_pose_name=%s, "
                "current_pose=(%f,%f,%f)",
                goal->map_name.c_str(), goal->target_node_id,
                goal->target_pose_name.c_str(), goal->current_pose.x,
                goal->current_pose.y, goal->current_pose.theta);

    // Check if follow path action is available
    if (!v_follow_path_client_->wait_for_action_server(
            std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Follow path action server is not available");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                    vrobot_route_follow::action::MoveToPose>>
                    goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received cancel goal");

    // Cancel all goals of follow path client
    RCLCPP_WARN(this->get_logger(), "Cancel action - follow path will stop");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                           vrobot_route_follow::action::MoveToPose>>
                           goal_handle) {

    execute(goal_handle);
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   vrobot_route_follow::action::MoveToPose>>
                   goal_handle) {

    const auto goal = goal_handle->get_goal();
    auto       feedback =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Feedback>();
    auto result =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();

    auto start_time = std::chrono::steady_clock::now();

    try {
      // Step 1: Load map data using modular architecture
      RCLCPP_INFO(this->get_logger(), "Loading map data for: %s",
                  goal->map_name.c_str());

      if (current_map_name_ != goal->map_name || !map_loaded_) {
        if (!loadMapData(goal->map_name)) {
          result->success = false;
          result->error_message = "Cannot load map data";
          goal_handle->abort(result);
          return;
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "Target pose name: %s, target node id: %lu",
                  goal->target_pose_name.c_str(), goal->target_node_id);

      auto                   target_node_id = goal->target_node_id;
      Mapper<amr_ros2::Node> mapperNodes(db_client_);
      Mapper<amr_ros2::Map>  mapperMaps(db_client_);

      // Find map by name
      auto maps = mapperMaps.findBy(Criteria(
          amr_ros2::Map::Cols::_map_name, CompareOperator::EQ, goal->map_name));
      if (maps.empty()) {
        throw std::runtime_error("Map " + goal->map_name + " not found");
      }
      auto map_record   = maps[0];
      auto map_id_value = *map_record.getIdMap();

      // Check if node name is in database with name and map_id
      if (goal->target_pose_name != "") {
        auto nodes = mapperNodes.findBy(
            Criteria(amr_ros2::Node::Cols::_map_id, // Column map_id
                     CompareOperator::EQ,           // Equal
                     map_id_value                   // Value 
                     ) &&
            Criteria(amr_ros2::Node::Cols::_node_name, // Column node_name
                     CompareOperator::EQ,              // Equal
                     goal->target_pose_name            // Value 
                     ));

        if (nodes.empty()) {
          result->success       = false;
          result->error_message = "Target node name not found in database";
          goal_handle->abort(result);
          return;
        }
        target_node_id = *nodes[0].getId();
      }

      // Check if target node exists in loaded data
      const auto& nodes = database_loader_->getNodes();
      auto node_id_int32 = static_cast<int32_t>(target_node_id);
      if (nodes.find(node_id_int32) == nodes.end()) {
        result->success = false;
        result->error_message = "Target node id not found in loaded map data";
        goal_handle->abort(result);
        return;
      }

      // Step 2: Path planning using new modular architecture
      RCLCPP_INFO(this->get_logger(), "Start path planning...");

      Eigen::Vector3d current_pose(goal->current_pose.x, 
                                   goal->current_pose.y, 
                                   goal->current_pose.theta);

      // Create planning request
      PlanningRequest planning_request;
      planning_request.start_pose = current_pose;
      planning_request.goal_node_id = node_id_int32;
      planning_request.request_id = "action_request_" + std::to_string(this->now().nanoseconds());
      planning_request.timestamp = std::chrono::steady_clock::now();

      // Execute path planning using RichPathPlanner
      auto rich_result = path_planner_->planPath(planning_request);

      if (!rich_result.success) {
        result->success = false;
        result->error_message = "Error in path planning: " + rich_result.errorMessage;
        goal_handle->abort(result);
        return;
      }

      // Validate the result
      auto validation_result = path_validator_->validatePath(rich_result);
      if (!validation_result.is_valid) {
        RCLCPP_WARN(this->get_logger(), "Path validation failed with %zu issues", 
                    validation_result.issues.size());
        // Continue with warnings, only abort on errors
        for (const auto& issue : validation_result.issues) {
          if (issue.severity == ValidationSeverity::ERROR) {
            result->success = false;
            result->error_message = "Path validation failed: " + issue.description;
            goal_handle->abort(result);
            return;
          }
        }
      }

      // Optimize the path
      rich_result = path_optimizer_->quickOptimize(rich_result, OptimizationObjective::BALANCED);

      RCLCPP_INFO(this->get_logger(), "Path planning successful, distance: %.2f",
                  rich_result.totalDistance);

      // Step 3: Convert RichPathResult to nav/vpath and execute
      executeRichPathResult(goal_handle, rich_result, start_time);
    } catch (const std::exception &e) {
      result->success       = false;
      result->error_message = std::string("Exception: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Error in execution: %s", e.what());
    }
  }

  // New method to execute RichPathResult using modular architecture
  void executeRichPathResult(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          vrobot_route_follow::action::MoveToPose>>          &goal_handle,
      const RichPathResult& rich_result,
      std::chrono::steady_clock::time_point start_time) {

    try {
      // Convert RichPathResult to nav_msgs::Path and vrobot_local_planner::Path
      auto nav_path = vrobot_route_follow::utils::DatabaseConverter::richPathToNavPath(
          rich_result, "map", this->now());
      auto vpath = vrobot_route_follow::utils::DatabaseConverter::richPathToVPath(
          rich_result, "map", this->now());

      // Publish paths for visualization
      path_publisher_->publish(nav_path);
      vpath_publisher_->publish(vpath);

      // Validate path before execution
      if (vpath.poses.empty()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Empty VPath generated from RichPathResult");
        auto result = std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success = false;
        result->error_message = "Empty VPath generated";
        goal_handle->abort(result);
        return;
      }

      // Execute the path using VFollowPath action
      auto follow_goal = vrobot_local_planner::action::VFollowPath::Goal();
      follow_goal.goal_checker_id = "stopped_goal_checker";
      follow_goal.controller_id = "";
      follow_goal.path = vpath;

      auto send_goal_options = rclcpp_action::Client<
          vrobot_local_planner::action::VFollowPath>::SendGoalOptions();

      // Result callback
      send_goal_options.result_callback = 
          [this, goal_handle, rich_result, start_time](
              const rclcpp_action::ClientGoalHandle<
                  vrobot_local_planner::action::VFollowPath>::WrappedResult &follow_result) {
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);

        auto result = std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();

        switch (follow_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result->success = true;
          result->error_message = "Path execution completed successfully";
          result->total_distance = rich_result.totalDistance;
          result->total_time = duration.count() / 1000.0;
          goal_handle->succeed(result);
          
          RCLCPP_INFO(this->get_logger(),
                      "🎉 Mission completed: %.3f m, %.2f s, algorithm: %s",
                      rich_result.totalDistance, result->total_time,
                      rich_result.algorithmUsed.c_str());
          break;

        case rclcpp_action::ResultCode::ABORTED:
          result->success = false;
          result->error_message = "Path execution aborted";
          result->total_distance = rich_result.totalDistance;
          goal_handle->abort(result);
          RCLCPP_ERROR(this->get_logger(), "❌ Path execution aborted");
          break;

        case rclcpp_action::ResultCode::CANCELED:
          result->success = false;
          result->error_message = "Path execution canceled";
          result->total_distance = rich_result.totalDistance;
          goal_handle->canceled(result);
          RCLCPP_WARN(this->get_logger(), "⚠️ Path execution canceled");
          break;

        default:
          result->success = false;
          result->error_message = "Path execution failed";
          result->total_distance = rich_result.totalDistance;
          goal_handle->abort(result);
          RCLCPP_ERROR(this->get_logger(), "❌ Path execution failed");
          break;
        }
      };

      // Feedback callback
      send_goal_options.feedback_callback =
          [goal_handle](rclcpp_action::ClientGoalHandle<
                  vrobot_local_planner::action::VFollowPath>::SharedPtr,
              const std::shared_ptr<const vrobot_local_planner::action::VFollowPath::Feedback>
                  follow_feedback) {
        
        auto feedback = std::make_shared<vrobot_route_follow::action::MoveToPose::Feedback>();
        feedback->distance_remaining = follow_feedback->distance_to_goal;
        feedback->speed = follow_feedback->speed;
        goal_handle->publish_feedback(feedback);
      };

      if (!v_follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(),
                     "VFollowPath action server not available");
        
        auto result = std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success = false;
        result->error_message = "VFollowPath action server not available";
        goal_handle->abort(result);
        return;
      }

      // Visualize rich path result
      auto rich_path_markers = RichVisualization::createRichPathMarkers(
          rich_result, "map", this->now(), true, true, true, !rich_result.velocityProfile.empty());
      pub_viz_->publish(rich_path_markers);

      RCLCPP_INFO(this->get_logger(), 
                  "🚀 Executing path: %zu nodes, %zu poses, %.2f m",
                  rich_result.nodeSequence.size(), 
                  rich_result.poseSequence.size(),
                  rich_result.totalDistance);

      v_follow_path_client_->async_send_goal(follow_goal, send_goal_options);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in executeRichPathResult: %s", e.what());
      auto result = std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success = false;
      result->error_message = "Error in path execution: " + std::string(e.what());
      goal_handle->abort(result);
    }
  }



private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MoveToPoseActionServer>();

  RCLCPP_INFO(action_server->get_logger(),
              "Move To Pose Action Server is running...");

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}