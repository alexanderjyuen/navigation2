// Copyright (c) 2024 Polymath Robotics, Inc.
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BOOL_SUBSCRIBER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BOOL_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The BoolSubscriber behavior is used to subscribe to a boolean topic
 * and post the boolean on the blackboard
 */
class BoolSubscriber : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BoolSubscriber
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  BoolSubscriber(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>(
        "default_bool",
        "the default bool to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "bool_selector",
        "the input topic name to select the bool"),

      BT::OutputPort<bool>(
        "selected_bool",
        "Selected bool by subscription")
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the bool_selector topic
   *
   * @param msg the message with the id of the bool_selector
   */
  void callbackBoolSubscribe(const std_msgs::msg::Bool::SharedPtr msg);


  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_selector_sub_;

  bool last_bool_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BOOL_SUBSCRIBER_HPP_
