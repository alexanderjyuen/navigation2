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

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"

#include "nav2_behavior_tree/plugins/action/bool_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nav2_behavior_tree
{

using std::placeholders::_1;

BoolSubscriber::BoolSubscriber(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);
  if (!getInput("default_bool", last_bool_)) {
    RCLCPP_INFO(node_->get_logger(), "default_bool is empty, setting default bool to false");
    last_bool_ = false;
  }

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  bool_selector_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_,
    qos,
    std::bind(&BoolSubscriber::callbackBoolSubscribe, this, _1));
}

BT::NodeStatus BoolSubscriber::tick()
{
  // This behavior always use the last boolean received from the topic input.
  // When no input is specified it uses the default boolean.
  // If the default boolean is not specified then we fail the node

  setOutput("selected_bool", last_bool_);

  return BT::NodeStatus::SUCCESS;
}

void
BoolSubscriber::callbackBoolSubscribe(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_bool_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "last_bool_: %i", last_bool_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<polymath::polymath_nav2_behavior_tree::BoolSubscriber>("BoolSubscriber");
}
