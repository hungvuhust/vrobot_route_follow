#include "Straightlink.h"
#include "models/Node.h"
#include <cstddef>
#include <drogon/HttpAppFramework.h>
#include <drogon/orm/DbClient.h>
#include <drogon/orm/Mapper.h>

// mrpt
#include <memory>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/CTicTac.h>

// mathplot
#include "matplotlibcpp.h"
#include <cmath>
#include <optional>
#include <rclcpp/utilities.hpp>
#include <utility>
#include <vector>

// New organized headers
#include "vrobot_route_follow/graph_pose.hpp"
#include "vrobot_route_follow/utils/visualization.hpp"
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace drogon;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::gui;
using namespace drogon::orm;
using namespace std;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node    = std::make_shared<rclcpp::Node>("test_readdb");
  auto pub_vis = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "graph_vis", 10);

  // Publisher cho nav path
  auto pub_path =
      node->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

  auto client = DbClient::newPgClient(
      "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345", 1);

  if (!client) {
    LOG_ERROR << "Failed to create database client";
    return -1;
  }

  Mapper<drogon_model::amr_01::amr_ros2::Straightlink> mapperLinks(client);
  Mapper<drogon_model::amr_01::amr_ros2::Node>         mapperNodes(client);

  std::vector<drogon_model::amr_01::amr_ros2::Straightlink> links = mapperLinks.findAll();
  std::vector<drogon_model::amr_01::amr_ros2::Node>         nodes = mapperNodes.findAll();

  if (links.empty() || nodes.empty()) {
    LOG_ERROR << "No data found in the database";
    return -1;
  }

  std::unordered_map<TNodeID, CPose2D>              nodes_poses;
  std::vector<std::tuple<TNodeID, TNodeID, double>> links_poses;
  nodes_poses.clear();
  links_poses.clear();

  links_poses.reserve(links.size());

  for (const auto &node : nodes) {
    CPose2D tmp_pose(*node.getX(), *node.getY(), *node.getTheta());
    nodes_poses[*node.getId()] = tmp_pose;
  }
  for (const auto &link : links) {
    CPose2D start  = nodes_poses[*link.getIdStart()];
    CPose2D stop   = nodes_poses[*link.getIdEnd()];
    double  weight = (stop - start).norm();
    links_poses.push_back({*link.getIdStart(), *link.getIdEnd(), weight});
  }

  // Create graph using new organized structure
  vrobot_route_follow::GraphPose<TNodeID, CPose2D> graph(nodes_poses,
                                                         links_poses);

  // Create visualization using new utilities with labels and directions
  auto vis_markers =
      graph.toMarkerArray("map", node->now(), 0.1, 0.05, true, true);
  while (pub_vis->get_subscription_count() == 0) {
    std::cout << "Waiting for subscriber to connect..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  pub_vis->publish(vis_markers);

  // Test with new organized interface
  std::cout << "=== Test with New Organized Structure ===" << std::endl;

  // Test case 1: Distant pose (should fail with current config)
  CPose2D testPose1(2.5, -0.5, 0.0);
  int     test_case = 3;
  std::cout << "\n--- Test 1: Distant Pose ---" << std::endl;
  std::cout << "Testing pose: (" << testPose1.x() << ", " << testPose1.y()
            << ")" << std::endl;

  // Use new high-level interface
  vrobot_route_follow::GraphPose<TNodeID, CPose2D>::PlanningConfig config;
  config.directThreshold = 0.3;
  config.maxLinkDistance = 0.5;
  config.enablePruning   = false;

  auto result = graph.planPath(testPose1, test_case, config);

  if (result.success) {
    std::cout << "Planning successful using: " << result.algorithmUsed
              << std::endl;
    std::cout << "   Total distance: " << result.totalDistance.value()
              << std::endl;
    std::cout << "   Path segments: " << result.pathSegments.size()
              << std::endl;

    // Print metadata
    std::cout << "   Metadata:" << std::endl;
    for (const auto &[key, value] : result.metadata) {
      std::cout << "     " << key << ": " << value << std::endl;
    }

    // Convert to nav path and publish
    auto nav_path =
        graph.planningResultToNavPath(result, "map", node->now(), 0.02);

    while (pub_path->get_subscription_count() == 0) {
      std::cout << "Waiting for subscriber to connect..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    pub_path->publish(nav_path);
    std::cout << "   Published nav path with " << nav_path.poses.size()
              << " poses" << std::endl;

    // Print path segments
    std::cout << "\nPath segments:" << std::endl;
    for (size_t i = 0; i < result.pathSegments.size(); ++i) {
      const auto &seg = result.pathSegments[i];
      std::cout << "  " << i + 1 << ": (" << seg.first.x() << ", "
                << seg.first.y() << ") -> (" << seg.second.x() << ", "
                << seg.second.y() << ")" << std::endl;
    }

    // Create colored path visualization
    std::vector<std::tuple<std::pair<CPose2D, CPose2D>, std::string,
                           std_msgs::msg::ColorRGBA>>
         coloredSegments;
    auto segmentColors =
        vrobot_route_follow::utils::Visualization<TNodeID,
                                                  CPose2D>::getSegmentColors();

    for (size_t i = 0; i < result.pathSegments.size(); ++i) {
      std::string segmentType = (i == 0) ? "approach"
                                : (i == result.pathSegments.size() - 1)
                                    ? "graph_nav"
                                    : "link_access";
      coloredSegments.emplace_back(result.pathSegments[i], segmentType,
                                   segmentColors[segmentType]);
    }

    auto colored_markers =
        graph.createColoredPathMarkers(coloredSegments, "map", node->now());
    while (pub_vis->get_subscription_count() == 0) {
      std::cout << "Waiting for subscriber to connect..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    pub_vis->publish(colored_markers);

  } else {
    std::cout << "❌ Planning failed: " << result.algorithmUsed << std::endl;
    if (!result.errorMessage.empty()) {
      std::cout << "   Reason: " << result.errorMessage << std::endl;
    }

    // Print debug info
    std::cout << "\n--- Debug Information ---" << std::endl;
    std::cout << "Graph statistics:" << std::endl;
    auto stats = graph.getGraphStatistics();
    for (const auto &[key, value] : stats) {
      std::cout << "  " << key << ": " << value << std::endl;
    }

    // Check closest nodes
    auto   closestNodeId     = graph.getClosestNode(testPose1);
    auto   closestNodePose   = graph.getNodePose(closestNodeId);
    double distanceToClosest = testPose1.distanceTo(closestNodePose);
    std::cout << "Closest node: " << closestNodeId << " at ("
              << closestNodePose.x() << ", " << closestNodePose.y() << ")"
              << ", distance: " << distanceToClosest << std::endl;

    // Debug closest links
    std::cout << "\nAnalyzing closest links:" << std::endl;
    graph.debugClosestLinks(testPose1, 5, config.maxLinkDistance);
  }

  rclcpp::shutdown();
  return 0;
}