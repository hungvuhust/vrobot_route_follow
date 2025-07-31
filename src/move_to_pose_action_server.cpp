#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "vrobot_route_follow/utils/simple_action_server.hpp"

// New modular architecture
#include "vrobot_route_follow/action/detail/move_to_pose__struct.hpp"
#include "vrobot_route_follow/core/rich_database_loader.hpp"
#include "vrobot_route_follow/core/rich_path_optimizer.hpp"
#include "vrobot_route_follow/core/rich_path_planner.hpp"
#include "vrobot_route_follow/core/rich_path_validator.hpp"
#include "vrobot_route_follow/utils/database_converter.hpp"
#include "vrobot_route_follow/utils/simple_action_server.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"

// Action messages
#include "vrobot_local_planner/action/follow_path.hpp"
#include "vrobot_local_planner/action/v_follow_path.hpp"
#include "vrobot_route_follow/action/move_to_pose.hpp"

// Geometry messages
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace drogon;
using namespace drogon::orm;
using namespace drogon_model::amr_01;

using namespace std::placeholders;

using namespace vrobot_route_follow::core;
using namespace vrobot_route_follow::utils;

class MoveToPoseActionServer : public rclcpp::Node {
private:
  std::shared_ptr<drogon::orm::DbClient> db_client_;

  using Action       = vrobot_route_follow::action::MoveToPose;
  using ActionServer = vrobot_route_follow::SimpleActionServer<Action>;
  std::unique_ptr<ActionServer> action_server_;

  rclcpp_action::Client<vrobot_local_planner::action::VFollowPath>::SharedPtr
      v_follow_path_client_;

  // Database components
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
  rclcpp::Publisher<vrobot_local_planner::msg::Path>::SharedPtr
      vpath_publisher_;

  // New modular architecture components - DISABLED FOR DEBUG
  std::shared_ptr<RichDatabaseLoader> database_loader_;
  std::shared_ptr<RichPathPlanner>    path_planner_;
  std::shared_ptr<RichPathOptimizer>  path_optimizer_;
  std::shared_ptr<RichPathValidator>  path_validator_;

  // Current loaded map
  std::string current_map_name_;
  bool        map_loaded_;

  std::chrono::steady_clock::time_point execution_start_time_;

public:
  explicit MoveToPoseActionServer(const std::string &connection_info)
      : rclcpp::Node("move_to_pose_action_server"), map_loaded_(false) {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(),
                "Move To Pose Action Server is starting...");

    db_client_ = drogon::orm::DbClient::newPgClient(connection_info, 1);

    // Initialize modular architecture components
    database_loader_ = std::make_shared<RichDatabaseLoader>(connection_info);
    path_planner_    = std::make_shared<RichPathPlanner>(database_loader_);
    path_optimizer_  = std::make_shared<RichPathOptimizer>();
    path_validator_  = std::make_shared<RichPathValidator>();

    // Create publisher for visualization
    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph_vis", rclcpp::QoS(10).transient_local().reliable());

    vpath_publisher_ = this->create_publisher<vrobot_local_planner::msg::Path>(
        "route_vpath", rclcpp::QoS(10).transient_local().reliable());

    // Create v follow path action client
    v_follow_path_client_ =
        rclcpp_action::create_client<vrobot_local_planner::action::VFollowPath>(
            this, "v_follow_path");

    // // Create SimpleActionServer
    action_server_ = std::make_unique<ActionServer>(
        this, "move_to_pose",
        std::bind(&MoveToPoseActionServer::executeGoal, this), nullptr,
        std::chrono::milliseconds(500), true);

    action_server_->activate();

    RCLCPP_INFO(this->get_logger(), "Move To Pose server is initialized");
  }

  ~MoveToPoseActionServer() {
    if (action_server_) {
      action_server_->deactivate();
    }
  }

private:
  bool loadMapData(const std::string &map_name) {
    try {
      // Use RichDatabaseLoader to load map data
      auto load_stats = database_loader_->loadMap(map_name);

      if (load_stats.nodes_loaded == 0 || load_stats.links_loaded == 0) {
        throw std::runtime_error("No data found for map: " + map_name);
      }

      current_map_name_ = map_name;
      map_loaded_       = true;

      RCLCPP_INFO(
          this->get_logger(),
          "Loaded map '%s' successfully: %zu nodes, %zu links (cache hit: %s)",
          map_name.c_str(), load_stats.nodes_loaded, load_stats.links_loaded,
          load_stats.cache_hit ? "true" : "false");

      // Visualize loaded map using new architecture
      const auto &nodes = database_loader_->getNodes();
      const auto &links = database_loader_->getLinks();

      auto map_markers = RichVisualization::createMapMarkers(
          nodes, links, database_loader_->getCurvedLinks(), "map", this->now(),
          0.1, 0.05);
      pub_viz_->publish(map_markers);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading map '%s': %s",
                   map_name.c_str(), e.what());
      map_loaded_ = false;
      return false;
    }

    return true;
  }

  void executeGoal() {
    RCLCPP_INFO(this->get_logger(), "Received new goal");

    // Get the current goal from SimpleActionServer
    const auto goal = action_server_->get_current_goal();

    RCLCPP_INFO(this->get_logger(),
                "Executing goal: map=%s, target_node=%lu, target_pose_name=%s, "
                "current_pose=(%f,%f,%f)",
                goal->map_name.c_str(), goal->target_node_id,
                goal->target_pose_name.c_str(), goal->current_pose.x,
                goal->current_pose.y, goal->current_pose.theta);

    // Check if follow path action is available
    if (!v_follow_path_client_->wait_for_action_server(
            std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Follow path action server is not available");
      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success       = false;
      result->error_message = "Follow path action server is not available";
      action_server_->terminate_all(result);
      return;
    }

    // Store current goal and start time for later use
    execution_start_time_ = std::chrono::steady_clock::now();

    // Execute the goal
    executeGoalInternal();
  }

  void executeGoalInternal() {
    const auto goal = action_server_->get_current_goal();

    auto feedback =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Feedback>();
    auto result =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();

    try {
      // Step 2: Load map data using modular architecture
      RCLCPP_INFO(this->get_logger(), "Loading map data for: %s",
                  goal->map_name.c_str());

      if (current_map_name_ != goal->map_name || !map_loaded_) {
        if (!loadMapData(goal->map_name)) {
          result->success       = false;
          result->error_message = "Cannot load map data";
          action_server_->terminate_all(result);
          return;
        }
      }

      RCLCPP_INFO(
          this->get_logger(), "Target pose name: %s, target node id: %lu",
          goal->target_pose_name.c_str() ? goal->target_pose_name.c_str()
                                         : "not specified",
          goal->target_node_id);

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
          action_server_->terminate_all(result);
          return;
        }
        target_node_id = *nodes[0].getId();
      }

      // Check if target node exists in loaded data
      const auto &nodes         = database_loader_->getNodes();
      auto        node_id_int32 = static_cast<int32_t>(target_node_id);
      if (nodes.find(node_id_int32) == nodes.end()) {
        result->success       = false;
        result->error_message = "Target node id not found in loaded map data";
        action_server_->terminate_all(result);
        return;
      }

      // Step 3: Path planning using new modular architecture
      RCLCPP_INFO(this->get_logger(), "Start path planning...");

      Eigen::Vector3d current_pose(goal->current_pose.x, goal->current_pose.y,
                                   goal->current_pose.theta);

      // Create planning request
      PlanningRequest planning_request;
      planning_request.start_pose   = current_pose;
      planning_request.goal_node_id = node_id_int32;
      planning_request.request_id =
          "action_request_" + std::to_string(this->now().nanoseconds());
      planning_request.timestamp = std::chrono::steady_clock::now();

      // Execute path planning using RichPathPlanner
      auto rich_result = path_planner_->planPath(planning_request);

      if (!rich_result.success) {
        result->success = false;
        result->error_message =
            "Error in path planning: " + rich_result.errorMessage;
        action_server_->terminate_all(result);
        return;
      }

      // Validate the result
      auto validation_result = path_validator_->validatePath(rich_result);
      if (!validation_result.is_valid) {
        RCLCPP_WARN(this->get_logger(),
                    "Path validation failed with %zu issues",
                    validation_result.issues.size());
        // Continue with warnings, only abort on errors
        for (const auto &issue : validation_result.issues) {
          if (issue.severity == ValidationSeverity::ERROR) {
            result->success = false;
            result->error_message =
                "Path validation failed: " + issue.description;
            action_server_->terminate_all(result);
            return;
          }
        }
      }

      // Optimize the path
      // rich_result = path_optimizer_->quickOptimize(
      //     rich_result, OptimizationObjective::BALANCED);

      RCLCPP_INFO(this->get_logger(),
                  "Path planning successful, distance: %.2f",
                  rich_result.totalDistance);

      // Step 4: Convert RichPathResult to nav/vpath and execute
      executeRichPathResult(rich_result);
    } catch (const std::exception &e) {
      result->success       = false;
      result->error_message = std::string("Exception: ") + e.what();
      action_server_->terminate_all(result);
      RCLCPP_ERROR(this->get_logger(), "Error in execution: %s", e.what());
    }
  }

  // New method to execute RichPathResult using modular architecture
  void executeRichPathResult(const RichPathResult &rich_result) {

    try {
      // Convert RichPathResult to nav_msgs::Path and vrobot_local_planner::Path
      auto vpath =
          vrobot_route_follow::utils::DatabaseConverter::richPathToVPath(
              rich_result, "map", this->now());

      // Publish paths for visualization
      vpath_publisher_->publish(vpath);

      // Validate path before execution
      if (vpath.poses.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "❌ Empty VPath generated from RichPathResult");
        auto result =
            std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success       = false;
        result->error_message = "Empty VPath generated";
        action_server_->terminate_all(result);
        return;
      }

      // Execute the path using VFollowPath action
      auto follow_goal = vrobot_local_planner::action::VFollowPath::Goal();
      follow_goal.goal_checker_id = "stopped_goal_checker";
      follow_goal.controller_id   = "";
      follow_goal.path            = vpath;

      auto send_goal_options = rclcpp_action::Client<
          vrobot_local_planner::action::VFollowPath>::SendGoalOptions();

      // Result callback
      send_goal_options.result_callback =
          [this, rich_result](
              const rclcpp_action::ClientGoalHandle<
                  vrobot_local_planner::action::VFollowPath>::WrappedResult
                  &follow_result) {
            auto end_time = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - execution_start_time_);

            auto result = std::make_shared<
                vrobot_route_follow::action::MoveToPose::Result>();

            switch (follow_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              result->success        = true;
              result->error_message  = "Path execution completed successfully";
              result->total_distance = rich_result.totalDistance;
              result->total_time     = duration.count() / 1000.0;
              action_server_->terminate_all(result);

              RCLCPP_INFO(this->get_logger(),
                          "🎉 Mission completed: %.3f m, %.2f s, algorithm: %s",
                          rich_result.totalDistance, result->total_time,
                          rich_result.algorithmUsed.c_str());
              break;

            case rclcpp_action::ResultCode::ABORTED:
              result->success        = false;
              result->error_message  = "Path execution aborted";
              result->total_distance = rich_result.totalDistance;
              action_server_->terminate_all(result);
              RCLCPP_ERROR(this->get_logger(), "❌ Path execution aborted");
              break;

            case rclcpp_action::ResultCode::CANCELED:
              result->success        = false;
              result->error_message  = "Path execution canceled";
              result->total_distance = rich_result.totalDistance;
              action_server_->terminate_all(result);
              RCLCPP_WARN(this->get_logger(), "⚠️ Path execution canceled");
              break;

            default:
              result->success        = false;
              result->error_message  = "Path execution failed";
              result->total_distance = rich_result.totalDistance;
              action_server_->terminate_all(result);
              RCLCPP_ERROR(this->get_logger(), "❌ Path execution failed");
              break;
            }
          };

      // Feedback callback
      send_goal_options.feedback_callback =
          [this](rclcpp_action::ClientGoalHandle<
                     vrobot_local_planner::action::VFollowPath>::SharedPtr,
                 const std::shared_ptr<
                     const vrobot_local_planner::action::VFollowPath::Feedback>
                     follow_feedback) {
            auto feedback = std::make_shared<
                vrobot_route_follow::action::MoveToPose::Feedback>();
            feedback->distance_remaining = follow_feedback->distance_to_goal;
            feedback->speed              = follow_feedback->speed;
            action_server_->publish_feedback(feedback);
          };

      if (!v_follow_path_client_->wait_for_action_server(
              std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(),
                     "VFollowPath action server not available");

        auto result =
            std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success       = false;
        result->error_message = "VFollowPath action server not available";
        action_server_->terminate_all(result);
        return;
      }

      // Visualize rich path result
      auto rich_path_markers = RichVisualization::createRichPathMarkers(
          rich_result, "map", this->now(), 0.15, 0.05);
      pub_viz_->publish(rich_path_markers);

      RCLCPP_INFO(this->get_logger(),
                  "🚀 Executing path: %zu nodes, %zu poses, %.2f m",
                  rich_result.nodeSequence.size(),
                  rich_result.poseSequence.size(), rich_result.totalDistance);

      v_follow_path_client_->async_send_goal(follow_goal, send_goal_options);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in executeRichPathResult: %s",
                   e.what());
      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success = false;
      result->error_message =
          "Error in path execution: " + std::string(e.what());
      action_server_->terminate_all(result);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MoveToPoseActionServer>(
      "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345");

  rclcpp::spin(action_server);
  rclcpp::shutdown();

  return 0;
}