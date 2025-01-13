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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_route_to_pose_action.hpp"

namespace nav2_behavior_tree
{

ComputeRouteToPoseAction::ComputeRouteToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputeRoute>(xml_tag_name, action_name, conf)
{
  RCLCPP_INFO(node_->get_logger(), "action_name %s", action_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "xml_tag_name %s", xml_tag_name.c_str());
}

//This is not a service that should be repeated over and over again, should only be done if there is a change in the goa

void ComputeRouteToPoseAction::on_tick()
{
  //TODO: Checks for undetermined combos? Or is that done at the route_server level

  
  goal_.use_start = false;
  if (getInput("goal_id", goal_.goal_id)){
    goal_.use_poses = false;
    if(getInput("start_id", goal_.start_id)){
      goal_.use_start = true;
    }
  } else if(getInput("goal", goal_.goal)){
    goal_.use_poses = true;
    if(getInput("start", goal_.start)){
      goal_.use_start = true;
    }
  } 
	 
}

BT::NodeStatus ComputeRouteToPoseAction::on_success()
{
  setOutput("route", result_.result->route);
  setOutput("path", result_.result->path); //This can go straight into the controller  
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeRouteToPoseAction::on_aborted()
{
  nav2_msgs::msg::Route empty_route;
  nav_msgs::msg::Path empty_path;
  setOutput("route", empty_route);
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRouteToPoseAction::on_cancelled()
{
  nav2_msgs::msg::Route empty_route;
  nav_msgs::msg::Path empty_path;
  setOutput("route", empty_route);
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteToPoseAction::halt()
{
  nav2_msgs::msg::Route empty_route;
  nav_msgs::msg::Path empty_path;
  setOutput("route", empty_route);
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeRouteToPoseAction>(
        name, "compute_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeRouteToPoseAction>(
    "ComputeRouteToPose", builder);
}
