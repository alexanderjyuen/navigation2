// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_

#include <string>
#include <set>
#include <vector>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

/**
 * @brief Parse XML string to geometry_msgs::msg::Point
 * @param key XML string
 * @return geometry_msgs::msg::Point
 */
template<>
inline geometry_msgs::msg::Point convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<geometry_msgs::msg::Point>(new_key);
  }

  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::msg::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::msg::Quaternion
 * @param key XML string
 * @return geometry_msgs::msg::Quaternion
 */
template<>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<geometry_msgs::msg::Quaternion>(new_key);
  }

  // four real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::msg::PoseStamped
 * @param key XML string
 * @return geometry_msgs::msg::PoseStamped
 */
template<>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<geometry_msgs::msg::PoseStamped>(new_key);
  }

  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 9) {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }
}

/**
 * @brief Parse XML string to std::vector<geometry_msgs::msg::PoseStamped>
 * @param key XML string
 * @return std::vector<geometry_msgs::msg::PoseStamped>
 */
template<>
inline std::vector<geometry_msgs::msg::PoseStamped> convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<std::vector<geometry_msgs::msg::PoseStamped>>(new_key);
  }

  auto parts = BT::splitString(key, ';');
  if (parts.size() % 9 != 0) {
    throw std::runtime_error("invalid number of fields for std::vector<PoseStamped> attribute)");
  } else {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (size_t i = 0; i < parts.size(); i += 9) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[i]));
      pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[i + 1]);
      pose_stamped.pose.position.x = BT::convertFromString<double>(parts[i + 2]);
      pose_stamped.pose.position.y = BT::convertFromString<double>(parts[i + 3]);
      pose_stamped.pose.position.z = BT::convertFromString<double>(parts[i + 4]);
      pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[i + 5]);
      pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[i + 6]);
      pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[i + 7]);
      pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[i + 8]);
      poses.push_back(pose_stamped);
    }
    return poses;
  }
}

/**
 * @brief Parse XML string to nav_msgs::msg::Goals
 * @param key XML string
 * @return nav_msgs::msg::Goals
 */
template<>
inline nav_msgs::msg::Goals convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<nav_msgs::msg::Goals>(new_key);
  }

  auto parts = BT::splitString(key, ';');
  if ((parts.size() - 2) % 9 != 0) {
    throw std::runtime_error("invalid number of fields for Goals attribute)");
  } else {
    nav_msgs::msg::Goals goals_array;
    goals_array.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    goals_array.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    for (size_t i = 2; i < parts.size(); i += 9) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[i]));
      pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[i + 1]);
      pose_stamped.pose.position.x = BT::convertFromString<double>(parts[i + 2]);
      pose_stamped.pose.position.y = BT::convertFromString<double>(parts[i + 3]);
      pose_stamped.pose.position.z = BT::convertFromString<double>(parts[i + 4]);
      pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[i + 5]);
      pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[i + 6]);
      pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[i + 7]);
      pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[i + 8]);
      goals_array.goals.push_back(pose_stamped);
    }
    return goals_array;
  }
}

/**
 * @brief Parse XML string to nav_msgs::msg::Path
 * @param key XML string
 * @return nav_msgs::msg::Path
 */
template<>
inline nav_msgs::msg::Path convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<nav_msgs::msg::Path>(new_key);
  }

  auto parts = BT::splitString(key, ';');
  if ((parts.size() - 2) % 9 != 0) {
    throw std::runtime_error("invalid number of fields for Path attribute)");
  } else {
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    path.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    for (size_t i = 2; i < parts.size(); i += 9) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      path.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[i]));
      pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[i + 1]);
      pose_stamped.pose.position.x = BT::convertFromString<double>(parts[i + 2]);
      pose_stamped.pose.position.y = BT::convertFromString<double>(parts[i + 3]);
      pose_stamped.pose.position.z = BT::convertFromString<double>(parts[i + 4]);
      pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[i + 5]);
      pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[i + 6]);
      pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[i + 7]);
      pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[i + 8]);
      path.poses.push_back(pose_stamped);
    }
    return path;
  }
}

/**
 * @brief Parse XML string to nav2_msgs::msg::WaypointStatus
 * @param key XML string
 * @return nav2_msgs::msg::WaypointStatus
 */
template<>
inline nav2_msgs::msg::WaypointStatus convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<nav2_msgs::msg::WaypointStatus>(new_key);
  }

  auto parts = BT::splitString(key, ';');
  if (parts.size() != 13) {
    throw std::runtime_error("invalid number of fields for WaypointStatus attribute)");
  } else {
    nav2_msgs::msg::WaypointStatus waypoint_status;
    waypoint_status.waypoint_status = BT::convertFromString<uint8_t>(parts[0]);
    waypoint_status.waypoint_index = BT::convertFromString<uint32_t>(parts[1]);
    waypoint_status.waypoint_pose.header.stamp =
      rclcpp::Time(BT::convertFromString<int64_t>(parts[2]));
    waypoint_status.waypoint_pose.header.frame_id = BT::convertFromString<std::string>(parts[3]);
    waypoint_status.waypoint_pose.pose.position.x = BT::convertFromString<double>(parts[4]);
    waypoint_status.waypoint_pose.pose.position.y = BT::convertFromString<double>(parts[5]);
    waypoint_status.waypoint_pose.pose.position.z = BT::convertFromString<double>(parts[6]);
    waypoint_status.waypoint_pose.pose.orientation.x = BT::convertFromString<double>(parts[7]);
    waypoint_status.waypoint_pose.pose.orientation.y = BT::convertFromString<double>(parts[8]);
    waypoint_status.waypoint_pose.pose.orientation.z = BT::convertFromString<double>(parts[9]);
    waypoint_status.waypoint_pose.pose.orientation.w = BT::convertFromString<double>(parts[10]);
    waypoint_status.error_code = BT::convertFromString<uint16_t>(parts[11]);
    waypoint_status.error_msg = BT::convertFromString<std::string>(parts[12]);
    return waypoint_status;
  }
}

/**
 * @brief Parse XML string to nav2_msgs::msg::WaypointStatus
 * @param key XML string
 * @return nav2_msgs::msg::WaypointStatus
 */
template<>
inline std::vector<nav2_msgs::msg::WaypointStatus> convertFromString(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<std::vector<nav2_msgs::msg::WaypointStatus>>(new_key);
  }

  auto parts = BT::splitString(key, ';');
  if (parts.size() % 13 != 0) {
    throw std::runtime_error("invalid number of fields for std::vector<WaypointStatus> attribute)");
  } else {
    std::vector<nav2_msgs::msg::WaypointStatus> wp_status_vector;
    for (size_t i = 0; i < parts.size(); i += 13) {
      nav2_msgs::msg::WaypointStatus wp_status;
      wp_status.waypoint_status = BT::convertFromString<uint8_t>(parts[i]);
      wp_status.waypoint_index = BT::convertFromString<uint32_t>(parts[i + 1]);
      wp_status.waypoint_pose.header.stamp =
        rclcpp::Time(BT::convertFromString<int64_t>(parts[i + 2]));
      wp_status.waypoint_pose.header.frame_id = BT::convertFromString<std::string>(parts[i + 3]);
      wp_status.waypoint_pose.pose.position.x = BT::convertFromString<double>(parts[i + 4]);
      wp_status.waypoint_pose.pose.position.y = BT::convertFromString<double>(parts[i + 5]);
      wp_status.waypoint_pose.pose.position.z = BT::convertFromString<double>(parts[i + 6]);
      wp_status.waypoint_pose.pose.orientation.x = BT::convertFromString<double>(parts[i + 7]);
      wp_status.waypoint_pose.pose.orientation.y = BT::convertFromString<double>(parts[i + 8]);
      wp_status.waypoint_pose.pose.orientation.z = BT::convertFromString<double>(parts[i + 9]);
      wp_status.waypoint_pose.pose.orientation.w = BT::convertFromString<double>(parts[i + 10]);
      wp_status.error_code = BT::convertFromString<uint16_t>(parts[i + 11]);
      wp_status.error_msg = BT::convertFromString<std::string>(parts[i + 12]);
      wp_status_vector.push_back(wp_status);
    }
    return wp_status_vector;
  }
}

/**
 * @brief Parse XML string to std::chrono::milliseconds
 * @param key XML string
 * @return std::chrono::milliseconds
 */
template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  // if string starts with "json:{", try to parse it as json
  if (StartWith(key, "json:")) {
    auto new_key = key;
    new_key.remove_prefix(5);
    return convertFromJSON<std::chrono::milliseconds>(new_key);
  }

  return std::chrono::milliseconds(std::stoul(key.data()));
}

/**
 * @brief Return parameter value from behavior tree node or ros2 parameter file.
 * @param node nav2::LifecycleNode::SharedPtr
 * @param param_name std::string
 * @param behavior_tree_node T2
 * @return <T1>
 */
template<typename T1, typename T2 = BT::TreeNode>
T1 deconflictPortAndParamFrame(
  nav2::LifecycleNode::SharedPtr node,
  std::string param_name,
  const T2 * behavior_tree_node)
{
  T1 param_value;
  bool param_from_input = behavior_tree_node->getInput(param_name, param_value).has_value();

  if constexpr (std::is_same_v<T1, std::string>) {
    // not valid if port doesn't exist or it is an empty string
    param_from_input &= !param_value.empty();
  }

  if (!param_from_input) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "Parameter '%s' not provided by behavior tree xml file, "
      "using parameter from ros2 parameter file",
      param_name.c_str());
    node->get_parameter(param_name, param_value);
    return param_value;
  } else {
    RCLCPP_DEBUG(
      node->get_logger(),
      "Parameter '%s' provided by behavior tree xml file",
      param_name.c_str());
    return param_value;
  }
}

/**
 * @brief Try reading an import port first, and if that doesn't work
 * fallback to reading directly the blackboard.
 * The blackboard must be passed explicitly because config() is private in BT.CPP 4.X
 *
 * @param bt_node node
 * @param blackboard the blackboard ovtained with node->config().blackboard
 * @param param_name std::string
 * @param behavior_tree_node the node
 * @return <T>
 */
template<typename T> inline
bool getInputPortOrBlackboard(
  const BT::TreeNode & bt_node,
  const BT::Blackboard & blackboard,
  const std::string & param_name,
  T & value)
{
  if (bt_node.getInput<T>(param_name, value)) {
    return true;
  }
  if (blackboard.get<T>(param_name, value)) {
    return true;
  }
  return false;
}

// Macro to remove boiler plate when using getInputPortOrBlackboard
#define getInputOrBlackboard(name, value) \
  getInputPortOrBlackboard(*this, *(this->config().blackboard), name, value);

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_
