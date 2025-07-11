// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include <chrono>
#include <thread>
#include <random>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critics/constraint_critic.hpp"
#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"
#include "nav2_mppi_controller/critics/goal_critic.hpp"
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_mppi_controller/critics/path_align_critic.hpp"
#include "nav2_mppi_controller/critics/path_angle_critic.hpp"
#include "nav2_mppi_controller/critics/path_follow_critic.hpp"
#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"
#include "nav2_mppi_controller/critics/twirling_critic.hpp"
#include "nav2_mppi_controller/critics/velocity_deadband_critic.hpp"
#include "utils_test.cpp"  // NOLINT

// Tests the various critic plugin functions

// ROS lock used from utils_test.cpp

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT

class PathAngleCriticWrapper : public PathAngleCritic
{
public:
  PathAngleCriticWrapper()
  : PathAngleCritic()
  {
  }

  void setMode(int mode)
  {
    mode_ = static_cast<PathAngleMode>(mode);
  }
};

TEST(CriticTests, ConstraintsCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  // provide velocities in constraints, should not have any costs
  state.vx = 0.40 * Eigen::ArrayXXf::Ones(1000, 30);
  state.vy = Eigen::ArrayXXf::Zero(1000, 30);
  state.wz = Eigen::ArrayXXf::Ones(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  ConstraintCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
  EXPECT_TRUE(critic.getMaxVelConstraint() > 0.0);
  EXPECT_TRUE(critic.getMinVelConstraint() < 0.0);

  // Scoring testing
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0, 1e-6);

  // provide out of maximum velocity constraint
  state.vx.row(999).setConstant(0.60f);
  critic.score(data);
  EXPECT_GT(costs.sum(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(999), 1.2, 0.01);
  costs.setZero();

  // provide out of minimum velocity constraint
  state.vx.row(1).setConstant(-0.45f);
  critic.score(data);
  EXPECT_GT(costs.sum(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(1), 1.2, 0.01);
  costs.setZero();

  // Now with ackermann, all in constraint so no costs to score
  state.vx.setConstant(0.40f);
  state.wz.setConstant(1.5f);
  data.motion_model = std::make_shared<AckermannMotionModel>(&param_handler, node->get_name());
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0, 1e-6);

  // Now violating the ackermann constraints
  state.wz.setConstant(2.5f);
  critic.score(data);
  EXPECT_GT(costs.sum(), 0);
  // 4.0 weight * 0.1 model_dt * (0.2 - 0.4/2.5) * 30 timesteps = 0.48
  EXPECT_NEAR(costs(1), 0.48, 0.01);
}

TEST(CriticTests, ObstacleCriticMisalignedParams) {
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", true);

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  ObstaclesCritic critic;
  // Expect throw when settings mismatched
  EXPECT_THROW(
    critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler),
    nav2_core::ControllerException
  );
}

TEST(CriticTests, ObstacleCriticAlignedParams) {
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", false);

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  ObstaclesCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
}


TEST(CriticTests, CostCriticMisAlignedParams) {
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
   auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", true);
  costmap_ros->on_configure(lstate);

  CostCritic critic;
  // Expect throw when settings mismatched
  EXPECT_THROW(
    critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler),
    nav2_core::ControllerException
  );
}

TEST(CriticTests, CostCriticAlignedParams) {
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
   auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", false);
  costmap_ros->on_configure(lstate);

  CostCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
}

TEST(CriticTests, GoalAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path too far from `threshold_to_consider` to consider
  state.pose.pose.position.x = 1.0;
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  path.yaws(9) = 3.14;
  goal.position.x = 10.0;
  goal.position.y = 0.0;
  goal.orientation.x = 0.0;
  goal.orientation.y = 0.0;
  goal.orientation.z = 1.0;
  goal.orientation.w = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0, 1e-6);

  // Let's move it even closer, just to be sure it still doesn't trigger
  state.pose.pose.position.x = 9.2;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0, 1e-6);

  // provide state pose and path below `threshold_to_consider` to consider
  state.pose.pose.position.x = 9.7;
  critic.score(data);
  EXPECT_GT(costs.sum(), 0);
  EXPECT_NEAR(costs(0), 9.42, 0.02);  // (3.14 - 0.0) * 3.0 weight
}

TEST(CriticTests, GoalCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing with all trajectories set to 0

  // provide state poses and path far, should not trigger
  state.pose.pose.position.x = 1.0;
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  goal.position.x = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 0.0, 1e-6);  // (0 * 5.0 weight
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);  // Should all be 0 * 1000
  costs.setZero();

  // provide state pose and path close
  path.x(9) = 0.5;
  path.y(9) = 0.0;
  goal.position.x = 0.5;
  critic.score(data);
  EXPECT_NEAR(costs(2), 2.5, 1e-6);  // (sqrt(10.0 * 10.0) * 5.0 weight
  EXPECT_NEAR(costs.sum(), 2500.0, 1e-3);  // should be 2.5 * 1000
}

TEST(CriticTests, PathAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PathAngleCriticWrapper critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close, within pose tolerance so won't do anything
  state.pose.pose.position.x = 0.0;
  state.pose.pose.position.y = 0.0;
  path.x(9) = 0.15;
  goal.position.x = 0.15;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with less than PI/2 angular diff.
  path.x(9) = 0.95;
  goal.position.x = 0.95;
  data.furthest_reached_path_point = 2;  // So it grabs the 2 + offset_from_furthest_ = 6th point
  path.x(6) = 1.0;  // angle between path point and pose = 0 < max_angle_to_furthest_
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path point and pose > max_angle_to_furthest_
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(costs.sum(), 0.0);
  EXPECT_NEAR(costs(0), 3.9947, 1e-2);  // atan2(4,-1) [1.81] * 2.2 weight

  // Set mode to no directional preferences + reset costs
  critic.setMode(1);
  costs.setZero();

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = 1.0;  // angle between path point and pose < max_angle_to_furthest_
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path pt and pose < max_angle_to_furthest_ IF non-directional
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path point and pose < max_angle_to_furthest_
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(costs.sum(), 0.0);
  // should use reverse orientation as the closer angle in no dir preference mode
  EXPECT_NEAR(costs(0), 2.9167, 1e-2);

  // Set mode to consider path directionality + reset costs
  critic.setMode(2);
  costs.setZero();

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = 1.0;  // angle between path point and pose < max_angle_to_furthest_
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path pt and pose < max_angle_to_furthest_ IF non-directional
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path point and pose < max_angle_to_furthest_
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(costs.sum(), 0.0);
  // should use reverse orientation as the closer angle in no dir preference mode
  EXPECT_NEAR(costs(0), 2.9167, 1e-2);

  PathAngleMode mode;
  mode = PathAngleMode::FORWARD_PREFERENCE;
  EXPECT_EQ(modeToStr(mode), std::string("Forward Preference"));
  mode = PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS;
  EXPECT_EQ(modeToStr(mode), std::string("Consider Feasible Path Orientations"));
  mode = PathAngleMode::NO_DIRECTIONAL_PREFERENCE;
  EXPECT_EQ(modeToStr(mode), std::string("No Directional Preference"));
  mode = static_cast<PathAngleMode>(4);
  EXPECT_EQ(modeToStr(mode), std::string("Invalid mode!"));
}

TEST(CriticTests, PreferForwardCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PreferForwardCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.x(9) = 10.0;
  goal.position.x = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0f, 1e-6f);

  // provide state pose and path close to trigger behavior but with all forward motion
  path.x(9) = 0.15;
  goal.position.x = 0.15;
  state.vx.setOnes();
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0f, 1e-6f);

  // provide state pose and path close to trigger behavior but with all reverse motion
  state.vx.setConstant(-1.0f);
  critic.score(data);
  EXPECT_GT(costs.sum(), 0.0f);
  EXPECT_NEAR(costs(0), 15.0f, 1e-3f);  // 1.0 * 0.1 model_dt * 5.0 weight * 30 length
}

TEST(CriticTests, TwirlingCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  TwirlingCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.x(9) = 10.0;
  goal.position.x = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path close to trigger behavior but with no angular variation
  path.x(9) = 0.15;
  goal.position.x = 0.15;
  state.wz.setZero();
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // Provide nearby with some motion
  state.wz.row(0).setConstant(10.0f);
  critic.score(data);
  EXPECT_NEAR(costs(0), 100.0, 1e-6);  // (mean(10.0) * 10.0 weight
  costs.setZero();

  // Now try again with some wiggling noise
  std::mt19937 engine;
  std::normal_distribution<float> normal_dist = std::normal_distribution(0.0f, 0.5f);
  state.wz.row(0) = Eigen::ArrayXf::NullaryExpr(30, [&] () {return normal_dist(engine);});
  critic.score(data);
  EXPECT_NEAR(costs(0), 2.581, 4e-1);  // (mean of noise with mu=0, sigma=0.5 * 10.0 weight
}

TEST(CriticTests, PathFollowCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(6);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  PathFollowCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and goal close within positional tolerances
  state.pose.pose.position.x = 2.0;
  path.x(5) = 1.8;
  goal.position.x = 1.8;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path far enough to enable
  // pose differential is (0, 0) and (0.15, 0)
  path.x(5) = 0.15;
  goal.position.x = 0.15;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 750.0, 1e-2);  // 0.15 * 5 weight * 1000
}

TEST(CriticTests, PathAlignCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  PathAlignCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.x(9) = 0.85;
  goal.position.x = 0.85;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path far enough to enable
  // but data furthest point reached is 0 and offset default is 20, so returns
  path.x(9) = 0.15;
  goal.position.x = 0.15;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // but with empty trajectories and paths, should still be zero
  *data.furthest_reached_path_point = 21;
  path.x(9) = 0.15;
  goal.position.x = 0.15;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // and with a valid path to pass invalid path condition
  state.pose.pose.position.x = 0.0;
  data.path_pts_valid.reset();  // Recompute on new path
  path.reset(22);
  path.x(0) = 0;
  path.x(1) = 0.1;
  path.x(2) = 0.2;
  path.x(3) = 0.3;
  path.x(4) = 0.4;
  path.x(5) = 0.5;
  path.x(6) = 0.6;
  path.x(7) = 0.7;
  path.x(8) = 0.8;
  path.x(9) = 0.9;
  path.x(10) = 0.9;
  path.x(11) = 0.9;
  path.x(12) = 0.9;
  path.x(13) = 0.9;
  path.x(14) = 0.9;
  path.x(15) = 0.9;
  path.x(16) = 0.9;
  path.x(17) = 0.9;
  path.x(18) = 0.9;
  path.x(19) = 0.9;
  path.x(20) = 0.9;
  path.x(21) = 0.9;
  goal.position.x = 0.9;
  generated_trajectories.x.setConstant(0.66f);
  critic.score(data);
  // 0.66 * 1000 * 10 weight * 6 num pts eval / 6 normalization term
  EXPECT_NEAR(costs.sum(), 6600.0, 1e-2);

  // provide state pose and path far enough to enable, with data to pass condition
  // but path is blocked in collision
  auto * costmap = costmap_ros->getCostmap();
  // island in the middle of lethal cost to cross. Costmap defaults to size 5x5 @ 10cm resolution
  for (unsigned int i = 11; i <= 30; ++i) {  // 1.1m-3m
    for (unsigned int j = 11; j <= 30; ++j) {  // 1.1m-3m
      costmap->setCost(i, j, 254);
    }
  }

  data.path_pts_valid.reset();  // Recompute on new path
  costs.setZero();
  path.x.setConstant(1.5f);
  path.y.setConstant(1.5f);
  goal.position.x = 1.5;
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0.0, 1e-6);
}

TEST(CriticTests, VelocityDeadbandCritic)
{
  // Standard preamble
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  auto getParam = param_handler.getParamGetter("critic");
  std::vector<double> deadband_velocities_;
  getParam(deadband_velocities_, "deadband_velocities", std::vector<double>{0.08, 0.08, 0.08});
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs = Eigen::ArrayXf::Zero(1000);
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};
  data.motion_model = std::make_shared<OmniMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  VelocityDeadbandCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide velocities out of deadband bounds, should not have any costs
  state.vx.setConstant(0.80f);
  state.vy.setConstant(0.60f);
  state.wz.setConstant(0.80f);
  critic.score(data);
  EXPECT_NEAR(costs.sum(), 0, 1e-6);

  // Test cost value
  state.vx.setConstant(0.01f);
  state.vy.setConstant(0.02f);
  state.wz.setConstant(0.021f);
  critic.score(data);
  // 35.0 weight * 0.1 model_dt * (0.07 + 0.06 + 0.059) * 30 timesteps = 56.7
  EXPECT_NEAR(costs(1), 19.845, 0.01);
}
