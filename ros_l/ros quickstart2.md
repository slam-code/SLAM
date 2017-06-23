http://wiki.ros.org/catkin/CMakeLists.txt
http://docs.ros.org/jade/api/catkin/html/howto/format1/catkin_library_dependencies.html
http://wiki.ros.org/ROS/Tutorials/CreatingPackage
https://stackoverflow.com/questions/30439692/writing-a-cmakelists-txt-for-ros

#1,创建工作区
# Create a new workspace in 'catkin_ws'.
mkdir catkin_ws
cd catkin_ws
wstool init src  #初始化工作区的源码目录

#2,切换到src中,创建一个package
catkin_create_pkg  ros_start0 # 创建ros_start的package
              该命令在ros_start0目录下创建了以下文件:
              CMakeLists.txt
              package.xml
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make

ros/ros.h: 没有那个文件或目录: catkin_create_pkg后面添加 depend2 roscpp

catkin_make是在src的上一级目录运行

source devel/setup.sh  #执行名为setup.bash的脚本
rosrun ros_start1 hello  #运行程序: rosrun PACKAGE EXECUTABLE

      [ INFO] [1497877153.878811522]: hello,... ROS !

#3发布者,publisher程序
       pubvel.cpp
       source devel/setup.sh
       rosrun ros_start1 pubvel
 
    [ INFO] [1497880074.238036618]: Sending random velocity command: linear=0.880616 angular=0.349486
    [ INFO] [1497880074.738053251]: Sending random velocity command: linear=0.0396733 angular=-0.288785
    [ INFO] [1497880075.238060871]: Sending random velocity command: linear=0.477793 angular=-0.791193 

#4订阅者,subscriber程序
    
    新cmd要重新运行 source devel/setup.sh
    [ INFO] [1497880060.877673196]:  position=(3.02,11.09) *direction=1.75
    [ INFO] [1497880060.893685226]:  position=(3.02,11.09) *direction=1.76
    [ INFO] [1497880060.909738085]:  position=(3.01,11.09) *direction=1.76
    [ INFO] [1497880060.925689618]:  position=(3.01,11.09) *direction=1.76




//http://m.blog.csdn.net/liulj95/article/details/47680599
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
http://blog.csdn.net/weifengdq/article/details/53220582

catkin_make
source devel/setup.bash
rospack find ros_start1

roslaunch ros_start1 example.launch