#include "vrobot_route_follow/utils/visualization.hpp"
#include <iostream>

namespace mrpt_graphPose_pose {
namespace utils {

// Explicit template instantiations for common types
template class Visualization<int, CPose2D, double>;
template class Visualization<long, CPose2D, double>;
template class Visualization<unsigned long, CPose2D, double>;

template <typename NodeID, typename Pose2D, typename WeightType>
visualization_msgs::msg::MarkerArray
Visualization<NodeID, Pose2D, WeightType>::toMarkerArray(
    const std::string &frameId, const rclcpp::Time &timestamp, double nodeScale,
    double edgeScale, bool showNodeLabels, bool showLinkDirections) const {

  visualization_msgs::msg::MarkerArray markerArray;

  if (Base::empty()) {
    return markerArray;
  }

  // Create node markers
  visualization_msgs::msg::Marker nodeMarker;
  nodeMarker.header.frame_id = frameId;
  nodeMarker.header.stamp    = timestamp;
  nodeMarker.ns              = "graph_nodes";
  nodeMarker.id              = 0;
  nodeMarker.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodeMarker.action          = visualization_msgs::msg::Marker::ADD;
  nodeMarker.scale.x         = nodeScale;
  nodeMarker.scale.y         = nodeScale;
  nodeMarker.scale.z         = nodeScale;
  nodeMarker.color.r         = 0.0;
  nodeMarker.color.g         = 1.0;
  nodeMarker.color.b         = 0.0;
  nodeMarker.color.a         = 1.0;

  // Add node positions
  for (const auto &[nodeId, pose] : Base::nodePoses_) {
    geometry_msgs::msg::Point point;
    point.x = pose.x();
    point.y = pose.y();
    point.z = 0.0;
    nodeMarker.points.push_back(point);
  }
  markerArray.markers.push_back(nodeMarker);

  // Create edge markers
  visualization_msgs::msg::Marker edgeMarker;
  edgeMarker.header.frame_id = frameId;
  edgeMarker.header.stamp    = timestamp;
  edgeMarker.ns              = "graph_edges";
  edgeMarker.id              = 1;
  edgeMarker.type            = visualization_msgs::msg::Marker::LINE_LIST;
  edgeMarker.action          = visualization_msgs::msg::Marker::ADD;
  edgeMarker.scale.x         = edgeScale;
  edgeMarker.color.r         = 1.0;
  edgeMarker.color.g         = 0.0;
  edgeMarker.color.b         = 0.0;
  edgeMarker.color.a         = 1.0;

  // Add edge lines
  for (const auto &[fromNode, neighbors] : Base::adjList_) {
    const auto &fromPose = Base::getNodePose(fromNode);

    for (const auto &[toNode, weight] : neighbors) {
      const auto &toPose = Base::getNodePose(toNode);

      geometry_msgs::msg::Point fromPoint, toPoint;
      fromPoint.x = fromPose.x();
      fromPoint.y = fromPose.y();
      fromPoint.z = 0.0;

      toPoint.x = toPose.x();
      toPoint.y = toPose.y();
      toPoint.z = 0.0;

      edgeMarker.points.push_back(fromPoint);
      edgeMarker.points.push_back(toPoint);
    }
  }
  markerArray.markers.push_back(edgeMarker);

  // Add node labels if requested
  if (showNodeLabels) {
    int labelId = 100; // Start label IDs from 100
    for (const auto &[nodeId, pose] : Base::nodePoses_) {
      visualization_msgs::msg::Marker labelMarker;
      labelMarker.header.frame_id = frameId;
      labelMarker.header.stamp    = timestamp;
      labelMarker.ns              = "node_labels";
      labelMarker.id              = labelId++;
      labelMarker.type    = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      labelMarker.action  = visualization_msgs::msg::Marker::ADD;
      labelMarker.scale.z = nodeScale * 2.0; // Text size
      labelMarker.color.r = 1.0;
      labelMarker.color.g = 1.0;
      labelMarker.color.b = 1.0;
      labelMarker.color.a = 1.0;

      labelMarker.pose.position.x = pose.x() + 0.1;
      labelMarker.pose.position.y = pose.y() + 0.1;
      labelMarker.pose.position.z = nodeScale * 2.0; // Above the node
      labelMarker.text            = std::to_string(nodeId);

      markerArray.markers.push_back(labelMarker);
    }
  }

  // Add link direction arrows if requested
  if (showLinkDirections) {
    int arrowId = 200; // Start arrow IDs from 200
    for (const auto &[fromNode, neighbors] : Base::adjList_) {
      const auto &fromPose = Base::getNodePose(fromNode);

      for (const auto &[toNode, weight] : neighbors) {
        const auto &toPose = Base::getNodePose(toNode);

        visualization_msgs::msg::Marker arrowMarker;
        arrowMarker.header.frame_id = frameId;
        arrowMarker.header.stamp    = timestamp;
        arrowMarker.ns              = "link_directions";
        arrowMarker.id              = arrowId++;
        arrowMarker.type            = visualization_msgs::msg::Marker::ARROW;
        arrowMarker.action          = visualization_msgs::msg::Marker::ADD;
        arrowMarker.scale.x         = edgeScale * 3.0; // Arrow length
        arrowMarker.scale.y         = edgeScale * 1.0; // Arrow width
        arrowMarker.scale.z         = edgeScale * 1.0; // Arrow height

        arrowMarker.color.r = 0.0;
        arrowMarker.color.g = 0.0;
        arrowMarker.color.b = 1.0;
        arrowMarker.color.a = 0.8;

        // Calculate arrow position and orientation
        double dx     = toPose.x() - fromPose.x();
        double dy     = toPose.y() - fromPose.y();
        double length = std::sqrt(dx * dx + dy * dy);

        if (length > 1e-6) {
          // Position arrow at 70% along the link
          double t                    = 0.7;
          arrowMarker.pose.position.x = fromPose.x() + t * dx;
          arrowMarker.pose.position.y = fromPose.y() + t * dy;
          arrowMarker.pose.position.z = 0.0;

          // Orient arrow in link direction
          double          yaw = std::atan2(dy, dx);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          arrowMarker.pose.orientation.x = q.x();
          arrowMarker.pose.orientation.y = q.y();
          arrowMarker.pose.orientation.z = q.z();
          arrowMarker.pose.orientation.w = q.w();

          markerArray.markers.push_back(arrowMarker);
        }
      }
    }
  }

  return markerArray;
}

template <typename NodeID, typename Pose2D, typename WeightType>
visualization_msgs::msg::MarkerArray
Visualization<NodeID, Pose2D, WeightType>::createPathMarkers(
    const std::vector<PathSegment> &pathSegments, const std::string &frameId,
    const rclcpp::Time &timestamp, const std_msgs::msg::ColorRGBA &pathColor,
    double lineWidth) const {

  visualization_msgs::msg::MarkerArray pathMarkers;

  if (pathSegments.empty()) {
    return pathMarkers;
  }

  visualization_msgs::msg::Marker pathMarker;
  pathMarker.header.frame_id = frameId;
  pathMarker.header.stamp    = timestamp;
  pathMarker.ns              = "planned_path";
  pathMarker.id              = 0;
  pathMarker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  pathMarker.action          = visualization_msgs::msg::Marker::ADD;
  pathMarker.scale.x         = lineWidth;
  pathMarker.color           = pathColor;

  // Add path points
  for (const auto &segment : pathSegments) {
    geometry_msgs::msg::Point startPoint;
    startPoint.x = segment.first.x();
    startPoint.y = segment.first.y();
    startPoint.z = 0.0;
    pathMarker.points.push_back(startPoint);
  }

  // Add final point
  if (!pathSegments.empty()) {
    geometry_msgs::msg::Point endPoint;
    endPoint.x = pathSegments.back().second.x();
    endPoint.y = pathSegments.back().second.y();
    endPoint.z = 0.0;
    pathMarker.points.push_back(endPoint);
  }

  pathMarkers.markers.push_back(pathMarker);
  return pathMarkers;
}

template <typename NodeID, typename Pose2D, typename WeightType>
visualization_msgs::msg::MarkerArray
Visualization<NodeID, Pose2D, WeightType>::createColoredPathMarkers(
    const std::vector<std::tuple<PathSegment, std::string,
                                 std_msgs::msg::ColorRGBA>> &coloredSegments,
    const std::string &frameId, const rclcpp::Time &timestamp,
    double lineWidth) const {

  visualization_msgs::msg::MarkerArray pathMarkers;

  for (size_t i = 0; i < coloredSegments.size(); ++i) {
    const auto &[segment, description, color] = coloredSegments[i];

    visualization_msgs::msg::Marker segmentMarker;
    segmentMarker.header.frame_id = frameId;
    segmentMarker.header.stamp    = timestamp;
    segmentMarker.ns              = "path_segment_" + description;
    segmentMarker.id              = static_cast<int>(i);
    segmentMarker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    segmentMarker.action          = visualization_msgs::msg::Marker::ADD;
    segmentMarker.scale.x         = lineWidth;
    segmentMarker.color           = color;

    geometry_msgs::msg::Point startPoint, endPoint;
    startPoint.x = segment.first.x();
    startPoint.y = segment.first.y();
    startPoint.z = 0.0;

    endPoint.x = segment.second.x();
    endPoint.y = segment.second.y();
    endPoint.z = 0.0;

    segmentMarker.points.push_back(startPoint);
    segmentMarker.points.push_back(endPoint);

    pathMarkers.markers.push_back(segmentMarker);
  }

  return pathMarkers;
}

template <typename NodeID, typename Pose2D, typename WeightType>
std_msgs::msg::ColorRGBA
Visualization<NodeID, Pose2D, WeightType>::createColor(double r, double g,
                                                       double b, double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

template <typename NodeID, typename Pose2D, typename WeightType>
std::map<std::string, std_msgs::msg::ColorRGBA>
Visualization<NodeID, Pose2D, WeightType>::getSegmentColors() {
  std::map<std::string, std_msgs::msg::ColorRGBA> colors;
  colors["approach"]    = createColor(0.0, 1.0, 1.0, 1.0); // Cyan
  colors["link_access"] = createColor(1.0, 1.0, 0.0, 1.0); // Yellow
  colors["link_follow"] = createColor(0.0, 1.0, 0.0, 1.0); // Green
  colors["graph_nav"]   = createColor(0.0, 0.0, 1.0, 1.0); // Blue
  colors["direct"]      = createColor(1.0, 0.0, 1.0, 1.0); // Magenta
  colors["fallback"]    = createColor(1.0, 0.5, 0.0, 1.0); // Orange
  return colors;
}

template <typename NodeID, typename Pose2D, typename WeightType>
visualization_msgs::msg::MarkerArray
Visualization<NodeID, Pose2D, WeightType>::createClearMarkers(
    const std::string &frameId, const rclcpp::Time &timestamp) {

  visualization_msgs::msg::MarkerArray clearMarkers;

  visualization_msgs::msg::Marker clearMarker;
  clearMarker.header.frame_id = frameId;
  clearMarker.header.stamp    = timestamp;
  clearMarker.action          = visualization_msgs::msg::Marker::DELETEALL;

  clearMarkers.markers.push_back(clearMarker);
  return clearMarkers;
}

template <typename NodeID, typename Pose2D, typename WeightType>
visualization_msgs::msg::MarkerArray
Visualization<NodeID, Pose2D, WeightType>::createLinkAnalysisMarkers(
    const Pose2D                                          &queryPose,
    const std::vector<std::tuple<NodeID, NodeID, double>> &closestLinks,
    const std::string &frameId, const rclcpp::Time &timestamp) const {

  visualization_msgs::msg::MarkerArray analysisMarkers;

  // Query pose marker
  visualization_msgs::msg::Marker queryMarker;
  queryMarker.header.frame_id = frameId;
  queryMarker.header.stamp    = timestamp;
  queryMarker.ns              = "query_pose";
  queryMarker.id              = 0;
  queryMarker.type            = visualization_msgs::msg::Marker::SPHERE;
  queryMarker.action          = visualization_msgs::msg::Marker::ADD;
  queryMarker.scale.x         = 0.2;
  queryMarker.scale.y         = 0.2;
  queryMarker.scale.z         = 0.2;
  queryMarker.color           = createColor(1.0, 0.0, 0.0, 1.0); // Red

  queryMarker.pose.position.x = queryPose.x();
  queryMarker.pose.position.y = queryPose.y();
  queryMarker.pose.position.z = 0.0;

  analysisMarkers.markers.push_back(queryMarker);

  // Link markers with distance labels
  for (size_t i = 0; i < closestLinks.size(); ++i) {
    const auto &[linkStart, linkEnd, distance] = closestLinks[i];

    // Link line marker
    visualization_msgs::msg::Marker linkMarker;
    linkMarker.header.frame_id = frameId;
    linkMarker.header.stamp    = timestamp;
    linkMarker.ns              = "analyzed_links";
    linkMarker.id              = static_cast<int>(i);
    linkMarker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    linkMarker.action          = visualization_msgs::msg::Marker::ADD;
    linkMarker.scale.x         = 0.08;

    // Color based on ranking (best = green, worst = red)
    double ratio = static_cast<double>(i) /
                   std::max(1.0, static_cast<double>(closestLinks.size() - 1));
    linkMarker.color = createColor(ratio, 1.0 - ratio, 0.0, 0.8);

    const auto &startPose = Base::getNodePose(linkStart);
    const auto &endPose   = Base::getNodePose(linkEnd);

    geometry_msgs::msg::Point startPoint, endPoint;
    startPoint.x = startPose.x();
    startPoint.y = startPose.y();
    startPoint.z = 0.0;

    endPoint.x = endPose.x();
    endPoint.y = endPose.y();
    endPoint.z = 0.0;

    linkMarker.points.push_back(startPoint);
    linkMarker.points.push_back(endPoint);

    analysisMarkers.markers.push_back(linkMarker);

    // Projection point marker (assuming projectOntoLink method exists)
    // Pose2D projection = this->projectOntoLink(queryPose, linkStart, linkEnd);

    visualization_msgs::msg::Marker projMarker;
    projMarker.header.frame_id = frameId;
    projMarker.header.stamp    = timestamp;
    projMarker.ns              = "projections";
    projMarker.id              = static_cast<int>(i);
    projMarker.type            = visualization_msgs::msg::Marker::CYLINDER;
    projMarker.action          = visualization_msgs::msg::Marker::ADD;
    projMarker.scale.x         = 0.1;
    projMarker.scale.y         = 0.1;
    projMarker.scale.z         = 0.05;
    projMarker.color           = createColor(0.0, 0.0, 1.0, 0.7); // Blue

    // Use midpoint as projection for now
    projMarker.pose.position.x = (startPose.x() + endPose.x()) / 2.0;
    projMarker.pose.position.y = (startPose.y() + endPose.y()) / 2.0;
    projMarker.pose.position.z = 0.0;

    analysisMarkers.markers.push_back(projMarker);
  }

  return analysisMarkers;
}

// Explicit instantiations for the template methods
template visualization_msgs::msg::MarkerArray
Visualization<int, CPose2D, double>::toMarkerArray(const std::string &,
                                                   const rclcpp::Time &, double,
                                                   double, bool, bool) const;

template visualization_msgs::msg::MarkerArray
Visualization<int, CPose2D, double>::createPathMarkers(
    const std::vector<PathSegment> &, const std::string &, const rclcpp::Time &,
    const std_msgs::msg::ColorRGBA &, double) const;

template visualization_msgs::msg::MarkerArray
Visualization<int, CPose2D, double>::createColoredPathMarkers(
    const std::vector<
        std::tuple<PathSegment, std::string, std_msgs::msg::ColorRGBA>> &,
    const std::string &, const rclcpp::Time &, double) const;

template std_msgs::msg::ColorRGBA
Visualization<int, CPose2D, double>::createColor(double, double, double,
                                                 double);

template std::map<std::string, std_msgs::msg::ColorRGBA>
Visualization<int, CPose2D, double>::getSegmentColors();

template visualization_msgs::msg::MarkerArray
Visualization<int, CPose2D, double>::createClearMarkers(const std::string &,
                                                        const rclcpp::Time &);

template visualization_msgs::msg::MarkerArray
Visualization<int, CPose2D, double>::createLinkAnalysisMarkers(
    const CPose2D &, const std::vector<std::tuple<int, int, double>> &,
    const std::string &, const rclcpp::Time &) const;

// Explicit instantiations for unsigned long (TNodeID type)
template visualization_msgs::msg::MarkerArray
Visualization<unsigned long, CPose2D, double>::toMarkerArray(
    const std::string &, const rclcpp::Time &, double, double, bool,
    bool) const;

template visualization_msgs::msg::MarkerArray
Visualization<unsigned long, CPose2D, double>::createPathMarkers(
    const std::vector<PathSegment> &, const std::string &, const rclcpp::Time &,
    const std_msgs::msg::ColorRGBA &, double) const;

template visualization_msgs::msg::MarkerArray
Visualization<unsigned long, CPose2D, double>::createColoredPathMarkers(
    const std::vector<
        std::tuple<PathSegment, std::string, std_msgs::msg::ColorRGBA>> &,
    const std::string &, const rclcpp::Time &, double) const;

template std_msgs::msg::ColorRGBA
Visualization<unsigned long, CPose2D, double>::createColor(double, double,
                                                           double, double);

template std::map<std::string, std_msgs::msg::ColorRGBA>
Visualization<unsigned long, CPose2D, double>::getSegmentColors();

template visualization_msgs::msg::MarkerArray
Visualization<unsigned long, CPose2D, double>::createClearMarkers(
    const std::string &, const rclcpp::Time &);

template visualization_msgs::msg::MarkerArray
Visualization<unsigned long, CPose2D, double>::createLinkAnalysisMarkers(
    const CPose2D &,
    const std::vector<std::tuple<unsigned long, unsigned long, double>> &,
    const std::string &, const rclcpp::Time &) const;

} // namespace utils
} // namespace mrpt_graphPose_pose