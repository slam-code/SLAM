# source /opt/ros/<distro>/setup.bash
source /opt/ros/indigo/setup.bash

创建工作空间:
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
编译
$ cd ~/catkin_ws/
$ catkin_make

catkin_make命令在catkin 工作空间中是一个非常方便的工具。如果你查看一下当前目录应该能看到'build'和'devel'这两个文件夹。
在'devel'文件夹里面你可以看到几个setup.*sh文件。source这些文件中的任何一个都可以将当前工作空间设置在ROS工作环境的最顶层，
想了解更多请参考catkin文档。接下来首先source一下新生成的setup.*sh文件：


$ source devel/setup.bash
要想保证工作空间已配置正确需确保ROS_PACKAGE_PATH环境变量包含你的工作空间目录，采用以下命令查看：


$ echo $ROS_PACKAGE_PATH

/opt/ros/kinetic/share


export | grep ROS
    declare -x ROSLISP_PACKAGE_DIRECTORIES=""
    declare -x ROS_DISTRO="kinetic"
    declare -x ROS_ETC_DIR="/opt/ros/kinetic/etc/ros"
    declare -x ROS_MASTER_URI="http://localhost:11311"
    declare -x ROS_PACKAGE_PATH="/opt/ros/kinetic/share"
    declare -x ROS_ROOT="/opt/ros/kinetic/share/ros"