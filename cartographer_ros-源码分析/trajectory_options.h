/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
#define CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/sensor_bridge.h"

namespace cartographer_ros {

//按照backpack_2d.lua配置,revo_lds.lua
struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  string tracking_frame;  //horizontal_laser_link
  string published_frame; //horizontal_laser_link
  string odom_frame;      //odom
  bool provide_odom_frame;//true
  bool use_odometry;      //false
  bool use_laser_scan;    //true
  bool use_multi_echo_laser_scan;//false
  int num_point_clouds;   //0
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
