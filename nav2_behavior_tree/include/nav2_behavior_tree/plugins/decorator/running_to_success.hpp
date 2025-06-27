// Copyright (c) 2025 Polymath Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUNNING_TO_SUCCESS_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUNNING_TO_SUCCESS_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that returns success if child node is RUNNING
 */
class RunningToSuccess : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RunningToSuccess
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RunningToSuccess(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts(){ return {};}

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUNNING_TO_SUCCESS_HPP_
