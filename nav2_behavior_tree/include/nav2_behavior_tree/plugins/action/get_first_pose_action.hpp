#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FIRST_POSE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FIRST_POSE_HPP_

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class GetFirstPose : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  typedef geometry_msgs::msg::PoseStamped Goal;

  GetFirstPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goal vector"),
      BT::OutputPort<Goal>("output_goal", "First goal in vector"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GET_FIRST_POSE_HPP_
