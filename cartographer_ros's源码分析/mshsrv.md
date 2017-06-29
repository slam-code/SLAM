/catkin_ws$ rossrv show cartographer_ros_msgs/FinishTrajectory 
int32 trajectory_id
---


catkin_ws$ rossrv show cartographer_ros_msgs/SubmapQuery 
int32 trajectory_id
int32 submap_index
---
int32 submap_version
uint8[] cells
int32 width
int32 height
float64 resolution
geometry_msgs/Pose slice_pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
string error_message