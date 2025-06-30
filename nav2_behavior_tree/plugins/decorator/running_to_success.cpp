// Copyright (c) 2019 Intel Corporation
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

#include <stdexcept>
#include <sstream>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/running_to_success.hpp"

namespace nav2_behavior_tree
{

RunningToSuccess::RunningToSuccess(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus RunningToSuccess::tick()
{

  setStatus(BT::NodeStatus::RUNNING);

  auto status = child_node_->executeTick();
  switch (status) {
    case BT::NodeStatus::FAILURE:
      return status;
    case BT::NodeStatus::SUCCESS:
      return status;
    case BT::NodeStatus::RUNNING:
      RCLCPP_INFO(node_->get_logger(), "RETURNING SUCCESS");
      return BT::NodeStatus::SUCCESS;
    default:
      std::stringstream error_msg;
      error_msg << "Invalid node status. Received status " << status <<
        "from child " << child_node_->name();
      throw std::runtime_error(error_msg.str());
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RunningToSuccess>("RunningToSuccess");
}
