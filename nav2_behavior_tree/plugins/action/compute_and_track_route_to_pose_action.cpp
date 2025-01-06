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

#include "nav2_behavior_tree/plugins/action/compute_and_track_route_to_pose_action.hpp"

namespace nav2_behavior_tree
{

ComputeAndTrackRouteToPoseAction::ComputeAndTrackRouteToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputeAndTrackRoute>(xml_tag_name, action_name, conf)
{
  RCLCPP_INFO(node_->get_logger(), "action_name %s", action_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "xml_tag_name %s", xml_tag_name.c_str());
}

//This is not a service that should be repeated over and over again, should only be done if there is a change in the goa

void ComputeAndTrackRouteToPoseAction::on_tick()
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

BT::NodeStatus ComputeAndTrackRouteToPoseAction::on_success()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeAndTrackRouteToPoseAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeAndTrackRouteToPoseAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputeAndTrackRouteToPoseAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

void ComputeAndTrackRouteToPoseAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::ComputeAndTrackRoute::Feedback> feedback)
{
  
  setOutput("last_node_id", feedback->last_node_id);
  setOutput("next_node_id", feedback->next_node_id); //TODO: Convert this to a goal for compute_path_to_pose
  setOutput("route", feedback->route);
  setOutput("path", feedback->path); //This can go straight into the controller  
  
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeAndTrackRouteToPoseAction>(
        name, "compute_and_track_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeAndTrackRouteToPoseAction>(
    "ComputeAndTrackRouteToPose", builder);
}
