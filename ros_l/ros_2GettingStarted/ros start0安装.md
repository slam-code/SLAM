#在Ubuntu中安装ROS Jade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
USTC (China)
 sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get update

桌面完整版安装：（推荐） 包含ROS、rqt、rviz、通用机器人函数库、2D/3D仿真器、导航以及2D/3D感知功能。
sudo apt-get install ros-jade-desktop-full

桌面版安装： 包含ROS、rqt、rviz以及通用机器人函数库。
sudo apt-get install ros-jade-desktop

基础版安装： 包含ROS核心软件包、构建工具以及通信相关的程序库，无GUI工具。
sudo apt-get install ros-jade-ros-base


单个软件包安装： 你也可以安装某个指定的ROS软件包（使用软件包名称替换掉下面的PACKAGE）：
sudo apt-get install ros-jade-PACKAGE
例如：
sudo apt-get install ros-jade-slam-gmapping

要查找可用软件包，请运行：

apt-cache search ros-jade

初始化 rosdep
在开始使用ROS之前你还需要初始化rosdep。rosdep可以方便在你需要编译某些源码的时候为其安装一些系统依赖，同时也是某些ROS核心功能组件所必需用到的工具。


sudo rosdep init
rosdep update

环境配置
如果每次打开一个新的终端时ROS环境变量都能够自动配置好（即添加到bash会话中），那将会方便很多：

echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc
如果你安装有多个ROS版本, ~/.bashrc 必须只能 source 你当前使用版本所对应的 setup.bash。

如果你只想改变当前终端下的环境变量，可以执行以下命令：

source /opt/ros/jade/setup.bash
安装 rosinstall
rosinstall 是ROS中一个独立分开的常用命令行工具，它可以方便让你通过一条命令就可以给某个ROS软件包下载很多源码树。

要在ubuntu上安装这个工具，请运行：


sudo apt-get install python-rosinstall