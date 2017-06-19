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

#ifndef CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_
#define CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "nav_msgs/OccupancyGrid.h"

namespace cartographer_ros {

/*

Occupancy grid mapping介绍:
https://wenku.baidu.com/view/a6167b82d5d8d15abe23482fb4daa58da0111c47.html
http://cn.mathworks.com/help/robotics/ug/occupancy-grids.html
https://github.com/ethz-asl/grid_map
https://v.qq.com/x/page/h0393e1kal1.html
http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html


*/
void BuildOccupancyGrid2D(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    ::nav_msgs::OccupancyGrid* const occupancy_grid);

// Computes MapLimits that contain the origin, and all rays (both returns and
// misses) in the 'trajectory_nodes'.
::cartographer::mapping_2d::MapLimits ComputeMapLimits(
    double resolution,
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_
