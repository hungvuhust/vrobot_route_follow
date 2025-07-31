#include <cmath>
#include <future>
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
#include "vrobot_local_planner/action/follow_path.hpp"
#include "vrobot_local_planner/action/v_follow_path.hpp"
#include "vrobot_route_follow/action/move_to_pose.hpp"

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

  rclcpp_action::Client<vrobot_local_planner::action::VFollowPath>::SharedPtr
      v_follow_path_client_;

  // Database components
  std::shared_ptr<drogon::orm::DbClient> db_client_;
  std::unique_ptr<
      vrobot_route_follow::GraphPose<TNodeID, CPose2D, double, double>>
                                                                     graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<vrobot_local_planner::msg::Path>::SharedPtr
      vpath_publisher_;

  // Cache to avoid reloading database every time
  std::unordered_map<TNodeID, CPose2D> nodes_poses_;
  std::vector<std::tuple<TNodeID, TNodeID, double, double>>
      links_poses_with_vel_;

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

    vpath_publisher_ = this->create_publisher<vrobot_local_planner::msg::Path>(
        "route_vpath", rclcpp::QoS(10).transient_local().reliable());

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

  bool loadGraphFromDatabase(const std::string &map_name) {
    try {

      Mapper<drogon_model::amr_01::amr_ros2::Straightlink> mapperLinks(
          db_client_);
      Mapper<drogon_model::amr_01::amr_ros2::Node> mapperNodes(db_client_);
      Mapper<drogon_model::amr_01::amr_ros2::Map>  mapperMaps(db_client_);

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
      links_poses_with_vel_.clear();
      links_poses_with_vel_.reserve(links.size());

      // Load nodes
      for (const auto &node : nodes) {
        CPose2D tmp_pose(*node.getX(), *node.getY(), *node.getTheta());
        nodes_poses_[*node.getId()] = tmp_pose;
      }

      // Load links
      for (const auto &link : links) {
        CPose2D start   = nodes_poses_[*link.getIdStart()];
        CPose2D stop    = nodes_poses_[*link.getIdEnd()];
        double  weight  = (stop - start).norm();
        double  max_vel = link.getMaxVelocity() ? *link.getMaxVelocity() : 2.0;
        links_poses_with_vel_.push_back(
            {*link.getIdStart(), *link.getIdEnd(), weight, max_vel});
      }

      // Create graph
      graph_ = std::make_unique<
          vrobot_route_follow::GraphPose<TNodeID, CPose2D, double, double>>(
          nodes_poses_, links_poses_with_vel_);

      RCLCPP_INFO(this->get_logger(),
                  "Loaded graph successfully: %zu nodes, %zu links for map: %s",
                  nodes_poses_.size(), links_poses_with_vel_.size(),
                  map_name.c_str());

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
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const vrobot_route_follow::action::MoveToPose::Goal>
          goal) {

    RCLCPP_INFO(
        this->get_logger(),
        "Received goal: map=%s, target_node=%lu, current_pose=(%f,%f,%f)",
        goal->map_name.c_str(), goal->target_node_id, goal->current_pose.x,
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
      vrobot_route_follow::GraphPose<TNodeID, CPose2D, double,
                                     double>::PlanningConfig config;
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

      // Step 2.5: Chia nhỏ đường đi để góc lệch không quá 80 độ
      auto path_segments = refinePathWithAngleConstraint(planning_result, 80.0);
      RCLCPP_INFO(this->get_logger(), "Path refined into %zu segments",
                  path_segments.size());

      // Tính tổng khoảng cách của tất cả segments
      double total_refined_distance = 0.0;
      for (const auto &segment : path_segments) {
        total_refined_distance += segment.totalDistance.value_or(0.0);
      }

      // Step 3: Follow từng path segment một cách tuần tự
      executePathSegments(goal_handle, path_segments, total_refined_distance,
                          start_time);

    } catch (const std::exception &e) {
      result->success       = false;
      result->error_message = std::string("Exception: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Error in execution: %s", e.what());
    }
  }

  // Function để chia nhỏ đường đi dựa trên góc lệch tối đa
  std::vector<vrobot_route_follow::GraphPose<TNodeID, CPose2D, double,
                                             double>::PlanningResult>
  refinePathWithAngleConstraint(
      const vrobot_route_follow::GraphPose<
          TNodeID, CPose2D, double, double>::PlanningResult &original_result,
      double max_angle_degrees) {

    std::vector<vrobot_route_follow::GraphPose<TNodeID, CPose2D, double,
                                               double>::PlanningResult>
        path_segments;

    if (original_result.pathSegments.size() <= 1) {
      path_segments.push_back(original_result);
      return path_segments; // Không cần refine nếu path quá ngắn
    }

    const double max_angle_rad = max_angle_degrees * M_PI / 180.0;

    // Chuyển pathSegments thành danh sách các điểm
    std::vector<CPose2D> path_points;
    if (!original_result.pathSegments.empty()) {
      path_points.push_back(original_result.pathSegments[0].first);
      for (const auto &segment : original_result.pathSegments) {
        path_points.push_back(segment.second);
      }
    }

    if (path_points.size() <= 2) {
      path_segments.push_back(original_result);
      return path_segments;
    }

    // Tìm các điểm chia cắt dựa trên góc lệch
    std::vector<size_t> split_indices;
    split_indices.push_back(0); // Luôn bắt đầu từ điểm đầu

    for (size_t i = 1; i < path_points.size() - 1; ++i) {
      const CPose2D &prev_pose = path_points[i - 1];
      const CPose2D &curr_pose = path_points[i];
      const CPose2D &next_pose = path_points[i + 1];

      // Tính vector hướng từ prev đến curr
      double dx1    = curr_pose.x() - prev_pose.x();
      double dy1    = curr_pose.y() - prev_pose.y();
      double angle1 = atan2(dy1, dx1);

      // Tính vector hướng từ curr đến next
      double dx2    = next_pose.x() - curr_pose.x();
      double dy2    = next_pose.y() - curr_pose.y();
      double angle2 = atan2(dy2, dx2);

      // Tính góc lệch
      double angle_diff = std::abs(angle2 - angle1);
      if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff; // Normalize về [0, π]
      }

      // Nếu góc lệch lớn hơn ngưỡng, tạo điểm chia cắt
      if (angle_diff > max_angle_rad) {
        split_indices.push_back(i);
        RCLCPP_DEBUG(this->get_logger(),
                     "Split at index %zu, angle diff: %.2f deg", i,
                     angle_diff * 180.0 / M_PI);
      }
    }

    split_indices.push_back(path_points.size() -
                            1); // Luôn kết thúc tại điểm cuối

    // Tạo các path segments riêng biệt
    for (size_t i = 0; i < split_indices.size() - 1; ++i) {
      size_t start_idx = split_indices[i];
      size_t end_idx   = split_indices[i + 1];

      vrobot_route_follow::GraphPose<TNodeID, CPose2D, double,
                                     double>::PlanningResult segment_result;
      segment_result.success       = true;
      segment_result.algorithmUsed = original_result.algorithmUsed + "_refined";

      // Tạo pathSegments cho segment này
      std::vector<std::pair<CPose2D, CPose2D>> segment_path;
      double                                   segment_distance = 0.0;

      // Log thông tin các điểm trong segment này
      std::string segment_points_info =
          "Segment " + std::to_string(i + 1) + " points: ";

      for (size_t j = start_idx; j < end_idx; ++j) {
        segment_path.emplace_back(path_points[j], path_points[j + 1]);
        segment_distance += (path_points[j + 1] - path_points[j]).norm();

        // Thêm thông tin tọa độ điểm
        segment_points_info += "(" + std::to_string(path_points[j].x()) + ", " +
                               std::to_string(path_points[j].y()) + ")";
        if (j < end_idx - 1)
          segment_points_info += " -> ";
      }
      // Thêm điểm cuối
      segment_points_info += " -> (" +
                             std::to_string(path_points[end_idx].x()) + ", " +
                             std::to_string(path_points[end_idx].y()) + ")";

      segment_result.pathSegments  = segment_path;
      segment_result.vpathSegments = graph_->pathSegmentsToVPath(segment_path);
      segment_result.totalDistance = segment_distance;

      // Copy metadata từ original result
      segment_result.metadata                  = original_result.metadata;
      segment_result.metadata["segment_index"] = static_cast<double>(i);
      segment_result.metadata["total_segments"] =
          static_cast<double>(split_indices.size() - 1);

      // Lưu thông tin điểm để log sau
      segment_result.metadata["points_info"] =
          static_cast<double>(i); // Dùng để identify

      path_segments.push_back(segment_result);

      RCLCPP_INFO(
          this->get_logger(),
          "Created segment %zu: from point %zu to %zu, distance: %.3f m", i + 1,
          start_idx, end_idx, segment_distance);
    }

    RCLCPP_INFO(this->get_logger(),
                "Path refined into %zu segments with max angle: %.1f deg",
                path_segments.size(), max_angle_degrees);

    return path_segments;
  }

  // Helper function để tìm node gần nhất với một pose
  std::pair<TNodeID, double> findNearestNode(const CPose2D &pose) const {
    TNodeID nearest_node = 0;
    double  min_distance = std::numeric_limits<double>::max();

    for (const auto &[node_id, node_pose] : nodes_poses_) {
      double distance = (pose - node_pose).norm();
      if (distance < min_distance) {
        min_distance = distance;
        nearest_node = node_id;
      }
    }

    return {nearest_node, min_distance};
  }

  // Function để log thông tin nodes trong một segment
  std::string getSegmentNodesInfo(
      const vrobot_route_follow::GraphPose<
          TNodeID, CPose2D, double, double>::PlanningResult &segment) const {
    std::string          nodes_info = "Nodes: ";
    std::vector<TNodeID> visited_nodes;

    if (!segment.pathSegments.empty()) {
      // Tìm node gần nhất với điểm đầu
      auto [start_node, start_dist] =
          findNearestNode(segment.pathSegments[0].first);
      visited_nodes.push_back(start_node);

      // Tìm nodes gần nhất với các điểm trung gian và cuối
      for (const auto &path_segment : segment.pathSegments) {
        auto [end_node, end_dist] = findNearestNode(path_segment.second);
        if (visited_nodes.empty() || visited_nodes.back() != end_node) {
          visited_nodes.push_back(end_node);
        }
      }
    }

    // Tạo string thông tin nodes
    for (size_t i = 0; i < visited_nodes.size(); ++i) {
      nodes_info += std::to_string(visited_nodes[i]);
      if (i < visited_nodes.size() - 1) {
        nodes_info += " -> ";
      }
    }

    return nodes_info;
  }

  // Function để execute từng path segment tuần tự
  void executePathSegments(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          vrobot_route_follow::action::MoveToPose>>          &goal_handle,
      const std::vector<vrobot_route_follow::GraphPose<
          TNodeID, CPose2D, double, double>::PlanningResult> &path_segments,
      double total_distance, std::chrono::steady_clock::time_point start_time) {

    // Tạo shared state để track tiến độ qua các segments
    auto shared_state             = std::make_shared<SegmentExecutionState>();
    shared_state->goal_handle     = goal_handle;
    shared_state->path_segments   = path_segments;
    shared_state->total_distance  = total_distance;
    shared_state->start_time      = start_time;
    shared_state->current_segment = 0;
    shared_state->completed_distance = 0.0;
    try {
      executeNextSegment(shared_state);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in executePathSegments: %s",
                   e.what());
      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success = false;
      result->error_message =
          "Error in executePathSegments: " + std::string(e.what());
      goal_handle->abort(result);
    }
  }

  // Struct để lưu trạng thái execution
  struct SegmentExecutionState {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vrobot_route_follow::action::MoveToPose>>
        goal_handle;
    std::vector<vrobot_route_follow::GraphPose<TNodeID, CPose2D, double,
                                               double>::PlanningResult>
                                          path_segments;
    double                                total_distance;
    std::chrono::steady_clock::time_point start_time;
    size_t                                current_segment;
    double                                completed_distance;
    bool received_feedback = false; // Track xem có nhận feedback không
    std::vector<size_t> skipped_segments; // Track segments bị skip
  };

  // Function để execute segment tiếp theo
  void executeNextSegment(std::shared_ptr<SegmentExecutionState> state) {
    bool is_last_segment = false;
    if (state->current_segment == state->path_segments.size() - 1) {
      is_last_segment = true;
    }

    if (state->current_segment >= state->path_segments.size()) {
      // Hoàn thành tất cả segments
      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      auto end_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - state->start_time);

      result->success        = true;
      result->error_message  = "All path segments completed successfully";
      result->total_distance = state->total_distance;
      result->total_time     = duration.count() / 1000.0;

      state->goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(),
                  "🎉 Mission completed: %zu segments, %.3f m, %.2f s",
                  state->path_segments.size(), state->total_distance,
                  result->total_time);

      return;
    }

    const auto &current_segment = state->path_segments[state->current_segment];

    // Reset feedback flag cho segment mới
    state->received_feedback = false;

    // Log thông tin chi tiết về segment
    std::string nodes_info = getSegmentNodesInfo(current_segment);

    RCLCPP_INFO(this->get_logger(), "Executing segment %zu/%zu: %s (%.3f m)",
                state->current_segment + 1, state->path_segments.size(),
                nodes_info.c_str(),
                current_segment.totalDistance.value_or(0.0));

    // Convert segment to nav_msgs::Path
    auto v_path = graph_->planningResultToVPath(current_segment, "map",
                                                this->now(), 0.02);
    vpath_publisher_->publish(v_path);

    auto nav_path = graph_->planningResultToNavPath(current_segment, "map",
                                                    this->now(), 0.02);
    path_publisher_->publish(nav_path);

    // Tạo follow path goal - thử với empty IDs trước
    auto follow_goal = vrobot_local_planner::action::VFollowPath::Goal();
    if (!is_last_segment) {
      follow_goal.goal_checker_id =
          "simple_goal_checker";      // Empty = use default
      follow_goal.controller_id = ""; // Empty = use default
    } else {
      follow_goal.goal_checker_id =
          "stopped_goal_checker";     // Empty = use default
      follow_goal.controller_id = ""; // Empty = use default
    }
    follow_goal.path = v_path;

    // Validate path trước khi gửi
    if (v_path.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Empty path for segment %zu",
                   state->current_segment + 1);

      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success       = false;
      result->error_message = "Empty path for segment " +
                              std::to_string(state->current_segment + 1);
      result->total_distance = state->completed_distance;
      state->goal_handle->abort(result);
      return;
    }

    auto send_goal_options = rclcpp_action::Client<
        vrobot_local_planner::action::VFollowPath>::SendGoalOptions();

    // Ghi lại thời gian bắt đầu gửi goal
    auto send_time = std::chrono::steady_clock::now();

    // Feedback callback
    send_goal_options.feedback_callback =
        [this,
         state](rclcpp_action::ClientGoalHandle<
                    vrobot_local_planner::action::VFollowPath>::SharedPtr,
                const std::shared_ptr<
                    const vrobot_local_planner::action::VFollowPath::Feedback>
                    follow_feedback) {
          // Đánh dấu đã nhận feedback
          if (!state->received_feedback) {
            state->received_feedback = true;
          }

          auto feedback = std::make_shared<
              vrobot_route_follow::action::MoveToPose::Feedback>();

          // Tính toán tiến độ tổng thể
          double segment_progress =
              1.0 - (follow_feedback->distance_to_goal /
                     state->path_segments[state->current_segment]
                         .totalDistance.value_or(1.0));
          double current_segment_distance =
              state->path_segments[state->current_segment]
                  .totalDistance.value_or(0.0);
          double current_completed =
              state->completed_distance +
              (segment_progress * current_segment_distance);

          feedback->distance_remaining =
              state->total_distance - current_completed;
          feedback->speed = follow_feedback->speed;

          state->goal_handle->publish_feedback(feedback);
        };

    // Result callback
    send_goal_options
        .result_callback = [this, state, send_time](
                               const rclcpp_action::ClientGoalHandle<
                                   vrobot_local_planner::action::VFollowPath>::
                                   WrappedResult &follow_result) {
      auto result_time = std::chrono::steady_clock::now();
      auto duration    = std::chrono::duration_cast<std::chrono::milliseconds>(
          result_time - send_time);

      switch (follow_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        // Segment thành công, chuyển sang segment tiếp theo
        {
          state->completed_distance +=
              state->path_segments[state->current_segment]
                  .totalDistance.value_or(0.0);

          RCLCPP_INFO(this->get_logger(), "✓ Segment %zu completed",
                      state->current_segment + 1);

          state->current_segment++;
          executeNextSegment(state);
          break;
        }

      case rclcpp_action::ResultCode::ABORTED: {
        auto result =
            std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success       = false;
        result->error_message = "Segment " +
                                std::to_string(state->current_segment + 1) +
                                " aborted";
        result->total_distance = state->completed_distance;
        state->goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Segment %zu aborted",
                     state->current_segment + 1);
      } break;

      case rclcpp_action::ResultCode::CANCELED: {
        auto result =
            std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success       = false;
        result->error_message = "Segment " +
                                std::to_string(state->current_segment + 1) +
                                " canceled";
        result->total_distance = state->completed_distance;
        state->goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Segment %zu canceled",
                    state->current_segment + 1);
      } break;

      default: {
        auto result =
            std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
        result->success = false;
        result->error_message =
            "Segment " + std::to_string(state->current_segment + 1) + " failed";
        result->total_distance = state->completed_distance;
        state->goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Segment %zu failed",
                     state->current_segment + 1);
      } break;
      }
    };

    if (!v_follow_path_client_->wait_for_action_server(
            std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Follow path action server not available for segment %zu",
                   state->current_segment + 1);

      auto result =
          std::make_shared<vrobot_route_follow::action::MoveToPose::Result>();
      result->success = false;
      result->error_message =
          "Follow path action server not available for segment " +
          std::to_string(state->current_segment + 1);
      result->total_distance = state->completed_distance;
      state->goal_handle->abort(result);
      return;
    }

    v_follow_path_client_->async_send_goal(follow_goal, send_goal_options);
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