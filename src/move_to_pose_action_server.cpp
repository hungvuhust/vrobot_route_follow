#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// Database and models
#include "models/Map.h"
#include "models/Node.h"
#include "models/Straightlink.h"
#include <drogon/HttpAppFramework.h>
#include <drogon/orm/DbClient.h>
#include <drogon/orm/Mapper.h>

// MRPT
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/CPose2D.h>

// Graph pose
#include "vrobot_route_follow/graph_pose.hpp"
#include "vrobot_route_follow/utils/nav_conversion.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"

// Action messages
#include "vrobot_route_follow/action/move_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

// Geometry messages
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace drogon;
using namespace drogon::orm;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace std::placeholders;

class MoveToPoseActionServer : public rclcpp::Node {
private:
  rclcpp_action::Server<vrobot_route_follow::action::MoveToPose>::SharedPtr
      action_server_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr
      follow_path_client_;

  // Database components
  std::shared_ptr<drogon::orm::DbClient>                             db_client_;
  std::unique_ptr<mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>>  graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // Cache to avoid reloading database every time
  std::unordered_map<TNodeID, CPose2D>              nodes_poses_;
  std::vector<std::tuple<TNodeID, TNodeID, double>> links_poses_;

public:
  MoveToPoseActionServer() : Node("move_to_pose_action_server") {

    // Initialize database client
    initDatabase();

    // Create publisher for visualization
    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph_vis", rclcpp::QoS(10).transient_local().reliable());

    // Create publisher for path
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "route_path", rclcpp::QoS(10).transient_local().reliable());

    // Create follow path action client
    follow_path_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
            this, "follow_path");

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
        "host=127.0.0.1 port=5432 dbname=vrobot user=vrobot password=1234", 1);

    if (!db_client_) {
      RCLCPP_ERROR(this->get_logger(), "Cannot connect to database");
      throw std::runtime_error("Database connection failed");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to database successfully");
  }

  bool loadGraphFromDatabase(const std::string &map_name) {
    try {

      Mapper<drogon_model::vrobot::Straightlink> mapperLinks(db_client_);
      Mapper<drogon_model::vrobot::Node>         mapperNodes(db_client_);
      Mapper<drogon_model::vrobot::Map>          mapperMaps(db_client_);

      // Find map by name
      auto maps = mapperMaps.findBy(
          Criteria("map_name", CompareOperator::EQ, map_name));
      if (maps.empty()) {
        throw std::runtime_error("Map " + map_name + " not found");
      }
      auto map_record   = maps[0];
      auto map_id_value = *map_record.getIdMap();

      // Load links and nodes
      auto links = mapperLinks.findBy(
          Criteria("map_id", CompareOperator::EQ, map_id_value));
      auto nodes = mapperNodes.findBy(
          Criteria("map_id", CompareOperator::EQ, map_id_value));

      if (links.empty() || nodes.empty()) {
        throw std::runtime_error("No data found in database for map: " +
                                 map_name);
      }

      // Clear previous data
      nodes_poses_.clear();
      links_poses_.clear();
      links_poses_.reserve(links.size());

      // Load nodes
      for (const auto &node : nodes) {
        CPose2D tmp_pose(*node.getX(), *node.getY(), *node.getTheta());
        nodes_poses_[*node.getId()] = tmp_pose;
      }

      // Load links
      for (const auto &link : links) {
        CPose2D start  = nodes_poses_[*link.getIdStart()];
        CPose2D stop   = nodes_poses_[*link.getIdStop()];
        double  weight = (stop - start).norm();
        links_poses_.push_back({*link.getIdStart(), *link.getIdStop(), weight});
      }

      // Create graph
      graph_ =
          std::make_unique<mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>>(
              nodes_poses_, links_poses_);

      RCLCPP_INFO(this->get_logger(),
                  "Loaded graph successfully: %zu nodes, %zu links for map: %s",
                  nodes_poses_.size(), links_poses_.size(), map_name.c_str());

      // Publish visualization
      auto vis_markers =
          graph_->toMarkerArray("map", this->now(), 0.1, 0.05, true, true);
      pub_viz_->publish(vis_markers);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading graph: %s", e.what());

      return false;
    }

    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID                                   &uuid,
      std::shared_ptr<const vrobot_route_follow::action::MoveToPose::Goal> goal) {

    RCLCPP_INFO(
        this->get_logger(),
        "Received goal: map=%s, target_node=%lu, current_pose=(%f,%f,%f)",
        goal->map_name.c_str(), goal->target_node_id, goal->current_pose.x,
        goal->current_pose.y, goal->current_pose.theta);

    // Check if follow path action is available
    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Follow path action server is not available");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<vrobot_route_follow::action::MoveToPose>>
          goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received cancel goal");

    // Cancel all goals of follow path client
    RCLCPP_WARN(this->get_logger(), "Cancel action - follow path will stop");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<vrobot_route_follow::action::MoveToPose>>
          goal_handle) {

    execute(goal_handle);
  }

  void
  execute(const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<vrobot_route_follow::action::MoveToPose>>
              goal_handle) {

    const auto goal = goal_handle->get_goal();
    auto       feedback =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Feedback>();
    auto result =
        std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();

    auto start_time = std::chrono::steady_clock::now();

    try {
      // Step 1: Load graph from database
      RCLCPP_INFO(this->get_logger(), "Loading graph for map: %s",
                  goal->map_name.c_str());

      if (!loadGraphFromDatabase(goal->map_name)) {
        result->success       = false;
        result->error_message = "Cannot load graph from database";
        goal_handle->abort(result);
        return;
      }

      // Check if target node exists
      if (nodes_poses_.find(goal->target_node_id) == nodes_poses_.end()) {
        result->success       = false;
        result->error_message = "Target node not found in graph";
        goal_handle->abort(result);
        return;
      }

      // Step 2: Calculate path planning
      RCLCPP_INFO(this->get_logger(), "Start path planning...");

      CPose2D currentPose(goal->current_pose.x, goal->current_pose.y,
                          goal->current_pose.theta);

      // Configure planning
      mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>::PlanningConfig config;
      config.directThreshold = 0.3;
      config.maxLinkDistance = 0.5;
      config.enablePruning   = false;

      // Thực hiện path planning
      auto planning_result =
          graph_->planPath(currentPose, goal->target_node_id, config);

      if (!planning_result.success) {
        result->success = false;
        result->error_message =
            "Error in path planning: " + planning_result.errorMessage;
        goal_handle->abort(result);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Path planning successful, distance: %f",
                  planning_result.totalDistance.value_or(0.0));

      // Convert to nav_msgs::Path
      auto nav_path = graph_->planningResultToNavPath(planning_result, "map",
                                                      this->now(), 0.02);
      path_publisher_->publish(nav_path);

      // Step 3: Call follow path action
      auto follow_goal            = nav2_msgs::action::FollowPath::Goal();
      follow_goal.goal_checker_id = "";
      follow_goal.controller_id   = "";
      follow_goal.path            = nav_path;

      auto send_goal_options = rclcpp_action::Client<
          nav2_msgs::action::FollowPath>::SendGoalOptions();

      // Callback when receiving feedback from follow path
      send_goal_options.feedback_callback =
          [goal_handle,
           feedback](rclcpp_action::ClientGoalHandle<
                         nav2_msgs::action::FollowPath>::SharedPtr,
                     const std::shared_ptr<
                         const nav2_msgs::action::FollowPath::Feedback>
                         follow_feedback) {
            // Forward feedback from follow path
            feedback->distance_remaining = follow_feedback->distance_to_goal;
            feedback->speed              = follow_feedback->speed;

            goal_handle->publish_feedback(feedback);
          };

      // Callback when follow path is finished
      send_goal_options.result_callback =
          [this, goal_handle, result, planning_result,
           start_time](const rclcpp_action::ClientGoalHandle<
                       nav2_msgs::action::FollowPath>::WrappedResult
                           &follow_result) {
            auto end_time = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time);

            result->total_distance =
                planning_result.totalDistance.value_or(0.0);
            result->total_time = duration.count() / 1000.0;

            switch (follow_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              result->success       = true;
              result->error_message = "Move successfully";
              goal_handle->succeed(result);
              RCLCPP_INFO(this->get_logger(), "Move successfully");
              break;

            case rclcpp_action::ResultCode::ABORTED:
              result->success       = false;
              result->error_message = "Follow path aborted";
              goal_handle->abort(result);
              break;

            case rclcpp_action::ResultCode::CANCELED:
              result->success       = false;
              result->error_message = "Follow path canceled";
              goal_handle->canceled(result);
              break;

            default:
              result->success       = false;
              result->error_message = "Follow path failed with unknown error";
              goal_handle->abort(result);
              break;
            }
          };

      // Send goal to follow path action
      RCLCPP_INFO(this->get_logger(), "Send goal to follow path action");
      follow_path_client_->async_send_goal(follow_goal, send_goal_options);
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for follow path action to finish");

    } catch (const std::exception &e) {
      result->success       = false;
      result->error_message = std::string("Exception: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Error in execution: %s", e.what());
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