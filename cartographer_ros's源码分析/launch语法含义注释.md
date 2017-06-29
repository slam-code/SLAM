<!--
  Copyright 2016 The Cartographer Authors

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  name="cartographer_node":别名
  pkg=" ":所属包
  <remap from="scan" to="horizontal_laser_2d">:将scan重映射到horizontal_laser_2d

  type="":package下的一个可执行文件(main函数)

-->

<launch>
  <param name="/use_sim_time" value="true" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename revo_lds.lua"
      output="screen">
    <remap from="scan" to="horizontal_laser_2d" />
  </node>

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
  <node name="playbag" pkg="rosbag" type="play"
      args="--clock $(arg bag_filename)" />
</launch>


//1:
rosrun rosbag play  cartographer_paper_revo_lds.bag

$rosbag info cartographer_paper_revo_lds.bag
path:         cartographer_paper_revo_lds.bag
version:      2.0
duration:     16:30s (990s)
start:        Jul 17 2015 17:06:59.42 (1437124019.42)
end:          Jul 17 2015 17:23:29.64 (1437125009.64)
size:         3.0 MB
messages:     4952
compression:  bz2 [19/19 chunks; 20.97%]
uncompressed: 14.2 MB @ 14.6 KB/s
compressed:    3.0 MB @  3.1 KB/s (20.97%)
types:        sensor_msgs/LaserScan [90c7ef2dc6895d81024acba2ac42f369]
topics:       horizontal_laser_2d   4952 msgs    : sensor_msgs/LaserScan




//2:原始:
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>

注释:

<launch> #以launch标签开头以表明这是一个launch文件。

  #命名空间（namespace)
  一个名为turtulesim1,使用相同的turtlesim节点并命名为'sim'。
  <group ns="turtlesim1">      
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

#另一个名为turtlesim2
  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

#并将所有话题的输入和输出分别重命名为turtlesim1和turtlesim2，(输出重定向)
  这样就会使turtlesim2模仿turtlesim1。(turtle1)


  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>


更详细文档:
http://wiki.ros.org/roslaunch/XML
http://wiki.ros.org/cn/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects