// Copyright (c) 2023, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_util/geometry_utils.hpp"

#ifndef NAV2_ROUTE__UTILS_HPP_
#define NAV2_ROUTE__UTILS_HPP_

namespace nav2_route
{

namespace utils
{

/**
 * @brief Convert the route graph into a visualization marker array for visualization
 * @param graph Graph of nodes and edges
 * @param frame Frame ID to use
 * @param now Current time to use
 * @return MarkerArray of the graph
 */
visualization_msgs::msg::MarkerArray toMsg(
  const Graph & graph, const std::string & frame, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker curr_marker;
  curr_marker.header.frame_id = frame;
  curr_marker.header.stamp = now;
  curr_marker.action = 0;

  auto getSphereSize = []() {
    geometry_msgs::msg::Vector3 msg;
    msg.x = 0.05;
    msg.y = 0.05;
    msg.z = 0.05;
    return msg;
  };

  auto getSphereColor = []() {
    std_msgs::msg::ColorRGBA msg;
    msg.r = 1.0;
    msg.g = 0.0;
    msg.b = 0.0;
    msg.a = 1.0;
    return msg;
  };

  auto getLineColor = []() {
    std_msgs::msg::ColorRGBA msg;
    msg.r = 0.0;
    msg.g = 1.0;
    msg.b = 0.0;
    msg.a = 0.5;  // So bi-directional connections stand out overlapping
    return msg;
  };

  unsigned int marker_idx = 1;
  for (unsigned int i = 0; i != graph.size(); i++) {
    curr_marker.ns = "route_graph";
    curr_marker.id = marker_idx++;
    curr_marker.type = visualization_msgs::msg::Marker::SPHERE;
    curr_marker.pose.position.x = graph[i].coords.x;
    curr_marker.pose.position.y = graph[i].coords.y;
    curr_marker.scale = getSphereSize();
    curr_marker.color = getSphereColor();
    msg.markers.push_back(curr_marker);

    // Add text
    curr_marker.ns = "route_graph_ids";
    curr_marker.id = marker_idx++;
    curr_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    curr_marker.pose.position.x = graph[i].coords.x + 0.07;
    curr_marker.pose.position.y = graph[i].coords.y;
    curr_marker.text = std::to_string(graph[i].nodeid);
    curr_marker.scale.z = 0.1;
    msg.markers.push_back(curr_marker);

    for (unsigned int j = 0; j != graph[i].neighbors.size(); j++) {
      curr_marker.ns = "route_graph";
      curr_marker.id = marker_idx++;
      curr_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      curr_marker.pose.position.x = 0;  // Set to 0 since points are relative to this frame
      curr_marker.pose.position.y = 0;  // Set to 0 since points are relative to this frame
      curr_marker.points.resize(2);
      curr_marker.points[0].x = graph[i].coords.x;
      curr_marker.points[0].y = graph[i].coords.y;
      curr_marker.points[1].x = graph[i].neighbors[j].end->coords.x;
      curr_marker.points[1].y = graph[i].neighbors[j].end->coords.y;
      curr_marker.scale.x = 0.03;
      curr_marker.color = getLineColor();
      msg.markers.push_back(curr_marker);
      curr_marker.points.clear();  // Reset for next node marker

      // Add text
      curr_marker.ns = "route_graph_ids";
      curr_marker.id = marker_idx++;
      curr_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      curr_marker.pose.position.x =
        graph[i].coords.x + ((graph[i].neighbors[j].end->coords.x - graph[i].coords.x) / 2.0)
        + 0.07;

      // Deal with overlapping bi-directional text markers by offsetting locations
      float y_offset = 0.0;
      if (graph[i].nodeid > graph[i].neighbors[j].end->nodeid) {
        y_offset = 0.05;
      } else {
        y_offset = -0.05;
      }

      curr_marker.pose.position.y =
        graph[i].coords.y + ((graph[i].neighbors[j].end->coords.y - graph[i].coords.y) / 2.0)
        + y_offset;
      curr_marker.text = std::to_string(graph[i].neighbors[j].edgeid);
      curr_marker.scale.z = 0.1;
      msg.markers.push_back(curr_marker);
    }
  }

  return msg;
}

}  // namespace utils

}  // namespace nav2_route

#endif  // NAV2_ROUTE__UTILS_HPP_