/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit_node.h"
#include "pure_pursuit_viz.h"
#include "util/planning_utils.h"
#include "util/tf_utils.h"

#include <iostream>


PurePursuitNode::PurePursuitNode() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<planning_utils::PurePursuit>();

  // Global Parameters
  private_nh_.param<double>("wheel_base", param_.wheel_base, 1.0);
  private_nh_.param<double>("target_speed", param_.target_speed, 0.5);
  private_nh_.param<double>("position_tolerance", param_.position_tolerance, 0.05); 

  // Node Parameters
  private_nh_.param<double>("control_period", param_.ctrl_period, 0.02);

  // Algorithm Parameters
  private_nh_.param<double>("lookahead_distance_ratio", param_.lookahead_distance_ratio, 2.2);
  private_nh_.param<double>("min_lookahead_distance", param_.min_lookahead_distance, 2.5);
  private_nh_.param<double>("lateral_error_threshold", param_.lateral_error_threshold, 0.1);

  // frame id
  private_nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  private_nh_.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
  // Lookahead frame moving along the path as the vehicle is moving.
  private_nh_.param<std::string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");

  // Subscribers
  sub_path_ = nh_.subscribe("path_segment", 1, &PurePursuitNode::onPath, this);
  sub_odom_ = nh_.subscribe("odometry", 1, &PurePursuitNode::onOdometry, this);

  // Publishers
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  pub_traj_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);

  // Debug Publishers
  pub_debug_marker_ = private_nh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 0);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(param_.ctrl_period), &PurePursuitNode::onTimer, this);

  // // Wait for first current pose
  // tf_utils::waitForTransform(tf_buffer_, "map", "base_link");

  is_velocity_set_ = false;

  is_pose_set_ = false;

  goal_reached_ = true;

  printParams();
}

void PurePursuitNode::printParams()
{
  ROS_INFO("wheel_base: %f", param_.wheel_base);
  ROS_INFO("target_speed: %f", param_.target_speed);
  ROS_INFO("position_tolerance: %f", param_.position_tolerance);
  ROS_INFO("control_period: %f", param_.ctrl_period);
  ROS_INFO("lookahead_distance_ratio: %f", param_.lookahead_distance_ratio);
  ROS_INFO("min_lookahead_distance: %f", param_.min_lookahead_distance);
  ROS_INFO("lateral_error_threshold: %f", param_.lateral_error_threshold);
}

bool PurePursuitNode::isDataReady() const
{
  if (!is_velocity_set_) {
    ROS_WARN_THROTTLE(5.0, "waiting for current velocity...");
    return false;
  }

  if (!is_pose_set_) {
    ROS_WARN_THROTTLE(5.0, "waiting for current_pose...");
    return false;
  }

  if (!path_) {
    ROS_WARN_THROTTLE(5.0, "waiting for path...");
    return false;
  }

  return true;
}

void PurePursuitNode::onOdometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 设置速度
  current_velocity_.linear.x = odom->twist.twist.linear.x;
  current_velocity_.linear.y = odom->twist.twist.linear.y;
  current_velocity_.angular.z = odom->twist.twist.angular.z;
  is_velocity_set_ = true;

  ROS_WARN_THROTTLE(5.0, "curr linear vel: %f, angular vel: %f", current_velocity_.linear.x, current_velocity_.angular.z);

  // 设置当前位姿
  try {
    geometry_msgs::TransformStamped tf;
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    current_pose_ = tf_utils::transform2pose(tf).pose;
    is_pose_set_ = true;

    // 用于显示
    trajectory_.push_back(tf_utils::transform2pose(tf));    

    ROS_WARN_THROTTLE(5.0, "curr pose:[%f, %f]", current_pose_.position.x, current_pose_.position.y);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

void PurePursuitNode::onPath(const nav_msgs::Path::ConstPtr& new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.

  ROS_WARN("frame_id: %s, map_frame_id: %s", new_path->header.frame_id.c_str(), map_frame_id_.c_str());
  
  if (new_path->header.frame_id == map_frame_id_)
  {
    // 清空之前的运行轨迹
    std::vector<geometry_msgs::PoseStamped>().swap(trajectory_);   
    if (new_path->poses.size() > 0)
    {
      path_ = new_path;
      goal_reached_ = false;
      ROS_WARN("Start:[%f, %f]", path_->poses.front().pose.position.x, path_->poses.front().pose.position.y);
      ROS_WARN("Goal:[%f, %f]", path_->poses.back().pose.position.x, path_->poses.back().pose.position.y);
    }
    else
    {
      goal_reached_ = true;
      ROS_WARN_STREAM("Received empty path!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
                    << " frame! Ignoring path in " << new_path->header.frame_id
                    << " frame!");
  }  
}

bool PurePursuitNode::reachedGoal()
{
  ROS_WARN_THROTTLE(5.0, "reaching Goal:[%f, %f]...", path_->poses.back().pose.position.x, path_->poses.back().pose.position.y);
  double ret = planning_utils::calcDistSquared2D(path_->poses.back().pose.position, current_pose_.position);
  return sqrt(ret) <= param_.position_tolerance ? true : false;
}

void PurePursuitNode::onTimer(const ros::TimerEvent & event)
{
  if (!isDataReady()) {
    return;
  }

  const auto target_values = calcTargetValues();

  if (target_values) {
    if (reachedGoal()) {
      ROS_WARN_THROTTLE(5.0, "Robot has reached goal! Stop!");
      publishCommand({0.0, 0.0});
    } else {
      publishCommand(*target_values);      
      // publishDebugMarker();
    }    
  } else {
    ROS_ERROR("failed to solve pure_pursuit");
    publishCommand({0.0, 0.0});
  }  

  is_pose_set_ = false;
  is_velocity_set_ = false;

  publishTrajectory();
}

void PurePursuitNode::publishCommand(const TargetValues & targets) 
{ 
  cmd_vel_.linear.x = targets.velocity;
  cmd_vel_.angular.z = targets.velocity * targets.kappa;
  // TODO: 最大角速度限制

  pub_vel_.publish(cmd_vel_);
}

boost::optional<TargetValues> PurePursuitNode::calcTargetValues() 
{
  // 固定速度
  const double target_vel = param_.target_speed;

  // 先按照速度计算一个前瞻距离
  const double min_lookahead_distance = param_.min_lookahead_distance;
  const double lookahead_distance = calcLookaheadDistance(
    current_velocity_.linear.x, param_.lookahead_distance_ratio, min_lookahead_distance);

  // 计算前方跟踪的曲率
  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(current_pose_);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(path_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);
  // pure_pursuit_->setClosestThreshold(3.0, M_PI);

  // Run PurePursuit
  auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  auto kappa = pure_pursuit_result.second;
  double lateral_err = pure_pursuit_->getLateralError();

  double adaptive_lookahead_distance = lookahead_distance;
  if (lateral_err > param_.lateral_error_threshold) {
    ROS_WARN("lateral_err:%f", lateral_err);
    // 根据横向误差自适应调整前瞻距离
    adaptive_lookahead_distance = sqrt(lookahead_distance*lookahead_distance + 2 * lookahead_distance * lateral_err + lateral_err * lateral_err);
  } else
  {
    // 根据曲率自适应调整前瞻距离
    adaptive_lookahead_distance = lookahead_distance / (1 + std::abs(kappa) * lookahead_distance);
  }

  adaptive_lookahead_distance = std::max(adaptive_lookahead_distance, min_lookahead_distance);


  // 重新设置前瞻距离
  pure_pursuit_->setLookaheadDistance(adaptive_lookahead_distance);
  pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  kappa = pure_pursuit_result.second;

  // Set debug data
  debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();

  return TargetValues{kappa, target_vel};
}

double PurePursuitNode::calcLookaheadDistance(
  double velocity, double lookahead_distance_ratio, double min_lookahead_distance)
{
  const double lookahead_distance = lookahead_distance_ratio * std::abs(velocity);
  return std::max(lookahead_distance, min_lookahead_distance);
}

void PurePursuitNode::publishDebugMarker() const
{
  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_));

  pub_debug_marker_.publish(marker_array);
}

void PurePursuitNode::publishTrajectory()
{
  nav_msgs::Path traj;
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id= map_frame_id_;

  for (auto pose : trajectory_) {
    traj.poses.push_back(pose);
  }

  pub_traj_.publish(traj);
}

