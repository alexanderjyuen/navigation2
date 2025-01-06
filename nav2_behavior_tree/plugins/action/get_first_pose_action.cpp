#include <string>
#include <memory>
#include <limits>

#include "nav2_behavior_tree/plugins/action/get_first_pose_action.hpp"

namespace nav2_behavior_tree
{

GetFirstPose::GetFirstPose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus GetFirstPose::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Goals goal_poses;
  getInput("input_goals", goal_poses);
  setOutput("output_goal", goal_poses[0]);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetFirstPose>("GetFirstPose");
}
