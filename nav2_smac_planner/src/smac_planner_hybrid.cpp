// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "nav2_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_ros(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  nav2::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2::declare_parameter_if_not_declared(
    node, name + ".terminal_checking_interval", rclcpp::ParameterValue(5000));
  node->get_parameter(name + ".terminal_checking_interval", _terminal_checking_interval);
  nav2::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  nav2::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
  nav2::declare_parameter_if_not_declared(
    node, name + ".allow_primitive_interpolation", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".allow_primitive_interpolation", _search_info.allow_primitive_interpolation);
  nav2::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  nav2::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  nav2::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_cost", rclcpp::ParameterValue(200.0));
  node->get_parameter(
    name + ".analytic_expansion_max_cost", _search_info.analytic_expansion_max_cost);
  nav2::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_cost_override", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".analytic_expansion_max_cost_override",
    _search_info.analytic_expansion_max_cost_override);
  nav2::declare_parameter_if_not_declared(
    node, name + ".use_quadratic_cost_penalty", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".use_quadratic_cost_penalty", _search_info.use_quadratic_cost_penalty);
  nav2::declare_parameter_if_not_declared(
    node, name + ".downsample_obstacle_heuristic", rclcpp::ParameterValue(true));
  node->get_parameter(
    name + ".downsample_obstacle_heuristic", _search_info.downsample_obstacle_heuristic);

  nav2::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  nav2::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

  nav2::declare_parameter_if_not_declared(
    node, name + ".debug_visualizations", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".debug_visualizations", _debug_visualizations);

  nav2::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
  // Note that we need to declare it here to prevent the parameter from being declared in the
  // dynamic reconfigure callback
  nav2::declare_parameter_if_not_declared(
    node, "introspection_mode", rclcpp::ParameterValue("disabled"));

  std::string goal_heading_type;
  nav2::declare_parameter_if_not_declared(
    node, name + ".goal_heading_mode", rclcpp::ParameterValue("DEFAULT"));
  node->get_parameter(name + ".goal_heading_mode", goal_heading_type);
  _goal_heading_mode = fromStringToGH(goal_heading_type);

  nav2::declare_parameter_if_not_declared(
    node, name + ".coarse_search_resolution", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".coarse_search_resolution", _coarse_search_resolution);

  if (_goal_heading_mode == GoalHeadingMode::UNKNOWN) {
    std::string error_msg = "Unable to get GoalHeader type. Given '" + goal_heading_type + "' "
      "Valid options are DEFAULT, BIDIRECTIONAL, ALL_DIRECTION. ";
    throw nav2_core::PlannerException(error_msg);
  }

  _motion_model = fromString(_motion_model_for_search);

  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  if (_coarse_search_resolution <= 0) {
    RCLCPP_WARN(
      _logger, "coarse iteration resolution selected as <= 0, "
      "disabling coarse iteration resolution search for goal heading"
    );

    _coarse_search_resolution = 1;
  }

  if (_angle_quantizations % _coarse_search_resolution != 0) {
    std::string error_msg = "coarse iteration should be an increment"
      " of the number of angular bins configured";
    throw nav2_core::PlannerException(error_msg);
  }

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    RCLCPP_WARN(
      _logger, "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

  // Initialize costmap downsampler
  _costmap_downsampler = std::make_unique<CostmapDownsampler>();
  std::string topic_name = "downsampled_costmap";
  _costmap_downsampler->on_configure(
    node, _global_frame, topic_name, _costmap, _downsampling_factor);

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan");

  if (_debug_visualizations) {
    _expansions_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("expansions");
    _planned_footprints_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "planned_footprints");
    _smoothed_footprints_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "smoothed_footprints");
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_debug_visualizations) {
    _expansions_publisher->on_activate();
    _planned_footprints_publisher->on_activate();
    _smoothed_footprints_publisher->on_activate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));

  // Special case handling to obtain resolution changes in global costmap
  auto resolution_remote_cb = [this](const rclcpp::Parameter & p) {
      dynamicParametersCallback(
        {rclcpp::Parameter("resolution", rclcpp::ParameterValue(p.as_double()))});
    };
  _remote_param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(_node.lock());
  _remote_resolution_handler = _remote_param_subscriber->add_parameter_callback(
    "resolution", resolution_remote_cb, "global_costmap/global_costmap");
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_debug_visualizations) {
    _expansions_publisher->on_deactivate();
    _planned_footprints_publisher->on_deactivate();
    _smoothed_footprints_publisher->on_deactivate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  // shutdown dyn_param_handler
  auto node = _node.lock();
  if (_dyn_params_handler && node) {
    node->remove_on_set_parameters_callback(_dyn_params_handler.get());
  }
  _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  nav2_smac_planner::NodeHybrid::destroyStaticAssets();
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
  if (_debug_visualizations) {
    _expansions_publisher.reset();
    _planned_footprints_publisher.reset();
    _smoothed_footprints_publisher.reset();
  }
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_downsample_costmap && _downsampling_factor > 1) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  float mx_start, my_start, mx_goal, my_goal;
  if (!costmap->worldToMapContinuous(
    start.pose.position.x,
    start.pose.position.y,
    mx_start,
    my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  _a_star->setStart(mx_start, my_start, static_cast<unsigned int>(orientation_bin));

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMapContinuous(
    goal.pose.position.x,
    goal.pose.position.y,
    mx_goal,
    my_goal))
  {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  _a_star->setGoal(mx_goal, my_goal, static_cast<unsigned int>(orientation_bin),
    _goal_heading_mode, _coarse_search_resolution);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Corner case of start and goal being on the same cell
  if (std::floor(mx_start) == std::floor(mx_goal) &&
    std::floor(my_start) == std::floor(my_goal))
  {
    pose.pose = start.pose;
    pose.pose.orientation = goal.pose.orientation;
    plan.poses.push_back(pose);

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
      _raw_plan_publisher->publish(plan);
    }

    return plan;
  }

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
  {
    if (_debug_visualizations) {
      geometry_msgs::msg::PoseArray msg;
      geometry_msgs::msg::Pose msg_pose;
      msg.header.stamp = _clock->now();
      msg.header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg.poses.push_back(msg_pose);
      }
      _expansions_publisher->publish(msg);
    }

    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      throw nav2_core::StartOccupied("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      throw nav2_core::NoValidPathCouldBeFound("no valid path found");
    } else {
      throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  if (_debug_visualizations) {
    // Publish expansions for debug
    auto now = _clock->now();
    geometry_msgs::msg::PoseArray msg;
    geometry_msgs::msg::Pose msg_pose;
    msg.header.stamp = now;
    msg.header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = getWorldOrientation(std::get<2>(e));
      msg.poses.push_back(msg_pose);
    }
    _expansions_publisher->publish(msg);

    if (_planned_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _planned_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _planned_footprints_publisher->publish(std::move(marker_array));
    }
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  if (_debug_visualizations) {
    if (_smoothed_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _smoothed_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      auto now = _clock->now();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _smoothed_footprints_publisher->publish(std::move(marker_array));
    }
  }

  return plan;
}

rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if(param_name.find(_name + ".") != 0 && param_name != "resolution") {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (param_name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (param_name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }

        if (parameter.as_double() < _costmap->getResolution() * _downsampling_factor) {
          RCLCPP_ERROR(
            _logger, "Min turning radius cannot be less than the search grid cell resolution!");
          result.successful = false;
        }

        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (param_name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      } else if (param_name == _name + ".analytic_expansion_max_cost") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_cost = static_cast<float>(parameter.as_double());
      } else if (param_name == "resolution") {
        // Special case: When the costmap's resolution changes, need to reinitialize
        // the controller to have new resolution information
        RCLCPP_INFO(_logger, "Costmap resolution changed. Reinitializing SmacPlannerHybrid.");
        reinit_collision_checker = true;
        reinit_a_star = true;
        reinit_downsampler = true;
        reinit_smoother = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (param_name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (param_name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (param_name == _name + ".allow_primitive_interpolation") {
        _search_info.allow_primitive_interpolation = parameter.as_bool();
        reinit_a_star = true;
      } else if (param_name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      } else if (param_name == _name + ".analytic_expansion_max_cost_override") {
        _search_info.analytic_expansion_max_cost_override = parameter.as_bool();
        reinit_a_star = true;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (param_name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (param_name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (param_name == _name + ".terminal_checking_interval") {
        reinit_a_star = true;
        _terminal_checking_interval = parameter.as_int();
      } else if (param_name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

        if (_angle_quantizations % _coarse_search_resolution != 0) {
          RCLCPP_WARN(
            _logger, "coarse iteration should be an increment of the "
            "number of angular bins configured. Disabling course research!"
          );
          _coarse_search_resolution = 1;
        }
      } else if (param_name == _name + ".coarse_search_resolution") {
        _coarse_search_resolution = parameter.as_int();
        if (_coarse_search_resolution <= 0) {
          RCLCPP_WARN(
            _logger, "coarse iteration resolution selected as <= 0. "
            "Disabling course research!"
          );
          _coarse_search_resolution = 1;
        }
        if (_angle_quantizations % _coarse_search_resolution != 0) {
          RCLCPP_WARN(
            _logger,
              "coarse iteration should be an increment of the "
              "number of angular bins configured. Disabling course research!"
          );
          _coarse_search_resolution = 1;
        }
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        _motion_model = fromString(parameter.as_string());
        if (_motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
            _motion_model_for_search.c_str());
        }
      } else if (param_name == _name + ".goal_heading_mode") {
        std::string goal_heading_type = parameter.as_string();
        GoalHeadingMode goal_heading_mode = fromStringToGH(goal_heading_type);
        RCLCPP_INFO(
          _logger,
          "GoalHeadingMode type set to '%s'.",
          goal_heading_type.c_str());
        if (goal_heading_mode == GoalHeadingMode::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get GoalHeader type. Given '%s', "
            "Valid options are DEFAULT, BIDIRECTIONAL, ALL_DIRECTION. ",
            goal_heading_type.c_str());
        } else {
          _goal_heading_mode = goal_heading_mode;
        }
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    auto node = _node.lock();

    // Re-Initialize A* template
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _terminal_checking_interval,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
        _costmap_downsampler->on_activate();
      }
    }

    // Re-Initialize collision checker
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_minimum_turning_radius_global_coords);
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
