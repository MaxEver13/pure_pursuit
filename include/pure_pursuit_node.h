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

#pragma once

#include <memory>
#include <vector>
#include <string>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "pure_pursuit.h"

struct Param
{
  // Global Parameters
  double wheel_base;
  double target_speed;
  double position_tolerance;

  // Node Parameters
  double ctrl_period;

  // Algorithm Parameters
  double lookahead_distance_ratio;
  double min_lookahead_distance;
  double lateral_error_threshold;  
};

struct TargetValues
{
  double kappa;
  double velocity;
};

struct DebugData
{
  geometry_msgs::Point next_target;
};

class PurePursuitNode
{
public:
  PurePursuitNode();

private:
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_path_;
  ros::Subscriber sub_odom_;

  std::string map_frame_id_, robot_frame_id_, lookahead_frame_id_;

  bool is_velocity_set_;
  bool is_pose_set_;
  geometry_msgs::Twist current_velocity_;
  nav_msgs::Path::ConstPtr path_;

  bool goal_reached_;


  bool isDataReady() const;

  void onPath(const nav_msgs::Path::ConstPtr& new_path);
  void onOdometry(const nav_msgs::Odometry::ConstPtr& odom);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;


  std::vector<geometry_msgs::PoseStamped> trajectory_;
  geometry_msgs::Pose current_pose_;

  // Publisher
  ros::Publisher pub_vel_;

  geometry_msgs::Twist cmd_vel_;

  void publishCommand(const TargetValues & targets);

  // Debug Publisher
  ros::Publisher pub_debug_marker_;

  void publishDebugMarker() const;

  // Trajectory Publisher

  ros::Publisher pub_traj_;

  void publishTrajectory();

  // Timer
  ros::Timer timer_;
  void onTimer(const ros::TimerEvent & event);

  // Parameter
  Param param_;

  // Controller
  std::unique_ptr<planning_utils::PurePursuit> pure_pursuit_;
  TargetValues target_values_;

  boost::optional<TargetValues> calcTargetValues();

  double calcLookaheadDistance(double velocity,
                              double lookahead_distance_ratio,
                              double min_lookahead_distance);

  bool reachedGoal();

  void printParams();

  // Debug
  mutable DebugData debug_data_;

  // Mutex


};
