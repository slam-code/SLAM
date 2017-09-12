编译ROS程序包
先source你的环境配置(setup)文件，在Ubuntu中的操作指令如下：
source /opt/ros/groovy/setup.bash
使用 catkin_make
# 在catkin工作空间下
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]

实例:
# 在一个CMake项目里
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (可选)

多个catkin项目可以放在工作空间中一起编译，工作流程如下：
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (可选)

ls
        build
        devel
        src
build 目录是build space的默认所在位置，同时cmake 和 make也是在这里被调用来配置并编译你的程序包。devel 目录是devel space的默认所在位置, 同时也是在你安装程序包之前存放可执行文件和库文件的地方。