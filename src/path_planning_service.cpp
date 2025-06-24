#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// Database và models
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

// ROS messages và services
#include "vrobot_route_follow/srv/path_planning.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace drogon;
using namespace drogon::orm;
using namespace mrpt::poses;
using namespace mrpt::graphs;

class PathPlanningService : public rclcpp::Node {
private:
  rclcpp::Service<vrobot_route_follow::srv::PathPlanning>::SharedPtr     service_;
  std::shared_ptr<drogon::orm::DbClient>                             db_client_;
  std::unique_ptr<mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>>  graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;

  // Cache để tránh reload database mỗi lần
  std::unordered_map<TNodeID, CPose2D>              nodes_poses_;
  std::vector<std::tuple<TNodeID, TNodeID, double>> links_poses_;
  bool                                              graph_loaded_;

public:
  PathPlanningService() : Node("path_planning_service"), graph_loaded_(false) {
    // Khởi tạo database client
    initDatabase();

    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph_vis", rclcpp::QoS(10).transient_local().reliable());

    // Load graph từ database
    loadGraphFromDatabase("");

    // Tạo service
    service_ = this->create_service<vrobot_route_follow::srv::PathPlanning>(
        "plan_path",
        std::bind(&PathPlanningService::handlePathPlanningRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Path Planning Service started");
  }

  ~PathPlanningService() {
    RCLCPP_INFO(this->get_logger(), "Path Planning Service stopped");
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

  bool loadGraphFromDatabase(std::string map_name) {
    try {
      Mapper<drogon_model::vrobot::Straightlink> mapperLinks(db_client_);
      Mapper<drogon_model::vrobot::Node>         mapperNodes(db_client_);
      Mapper<drogon_model::vrobot::Map>          mapperMaps(db_client_);
      // get id of map have name map_name
      auto                                       maps = mapperMaps.findBy(
                                                Criteria("map_name", CompareOperator::EQ, map_name));
      if (maps.empty()) {
        throw std::runtime_error("Map " + map_name + " not found");
      }
      auto map_record   = maps[0];
      auto map_id_value = *map_record.getIdMap();

      auto links = mapperLinks.findBy(
          Criteria("map_id", CompareOperator::EQ, map_id_value));
      auto nodes = mapperNodes.findBy(
          Criteria("map_id", CompareOperator::EQ, map_id_value));

      if (links.empty() || nodes.empty()) {
        throw std::runtime_error("No data found in database");
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

      // Tạo graph
      graph_ =
          std::make_unique<mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>>(
              nodes_poses_, links_poses_);

      graph_loaded_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Loaded graph successfully: %zu nodes, %zu links",
                  nodes_poses_.size(), links_poses_.size());
      auto vis_markers =
          graph_->toMarkerArray("map", this->now(), 0.1, 0.05, true, true);

      pub_viz_->publish(vis_markers);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading graph: %s", e.what());
      graph_loaded_ = false;
      return false;
    }

    return true;
  }

  void handlePathPlanningRequest(
      const std::shared_ptr<vrobot_route_follow::srv::PathPlanning::Request>
                                                                    request,
      std::shared_ptr<vrobot_route_follow::srv::PathPlanning::Response> response) {

    RCLCPP_INFO(this->get_logger(),
                "Received path planning request: target_node=%lu, "
                "current_pose=(%f,%f,%f) of map %s",
                request->target_node_id, request->current_pose.x,
                request->current_pose.y, request->current_pose.theta,
                request->map_name.c_str());

    graph_loaded_ = false;
    loadGraphFromDatabase(request->map_name);

    // Kiểm tra graph đã load chưa
    if (!graph_loaded_ || !graph_) {
      response->success        = false;
      response->error_message  = "Graph not loaded from database";
      response->algorithm_used = "none";
      RCLCPP_ERROR(this->get_logger(), "Graph not ready");
      return;
    }

    // Kiểm tra target node có tồn tại không
    if (nodes_poses_.find(request->target_node_id) == nodes_poses_.end()) {
      response->success        = false;
      response->error_message  = "Target node not found in graph";
      response->algorithm_used = "none";
      RCLCPP_ERROR(this->get_logger(), "Target node %lu not found",
                   request->target_node_id);
      return;
    }

    try {
      // Chuyển đổi current pose
      CPose2D currentPose(request->current_pose.x, request->current_pose.y,
                          request->current_pose.theta);

      // Cấu hình planning
      mrpt_graphPose_pose::GraphPose<TNodeID, CPose2D>::PlanningConfig config;
      config.directThreshold = 0.3;
      config.maxLinkDistance = 0.5;
      config.enablePruning   = false;

      // Thực hiện path planning
      auto result =
          graph_->planPath(currentPose, request->target_node_id, config);

      // Xử lý kết quả
      response->success        = result.success;
      response->algorithm_used = result.algorithmUsed;
      response->error_message  = result.errorMessage;

      if (result.success && result.totalDistance.has_value()) {
        response->total_distance = result.totalDistance.value();

        // Chuyển đổi thành nav_msgs::Path
        response->path =
            graph_->planningResultToNavPath(result, "map", this->now(), 0.02);

        RCLCPP_INFO(
            this->get_logger(),
            "Path planning successfully: algorithm=%s, distance=%f, poses=%zu",
            result.algorithmUsed.c_str(), result.totalDistance.value(),
            response->path.poses.size());
      } else {
        response->total_distance = 0.0;
        RCLCPP_WARN(this->get_logger(), "Path planning failed: %s - %s",
                    result.algorithmUsed.c_str(), result.errorMessage.c_str());
      }

    } catch (const std::exception &e) {
      response->success        = false;
      response->error_message  = std::string("Exception: ") + e.what();
      response->algorithm_used = "exception";
      RCLCPP_ERROR(this->get_logger(), "Exception in path planning: %s",
                   e.what());
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