预备工作
本教程中我们将会用到ros-tutorials程序包，请先安装：

$ sudo apt-get install ros-<distro>-ros-tutorials
将 <distro> 替换成你所安装的版本（比如Jade、Indigo、hydro、groovy、fuerte等）。

#1,文件系统概念
Packages: 软件包，是ROS应用程序代码的组织单元，
每个软件包都可以包含程序库、可执行文件、脚本或者其它手动创建的东西。
Manifest (package.xml): 清单，是对于'软件包'相关信息的描述,用于定义软件包相关元信息之间的依赖关系，这些信息包括版本、维护者和许可协议等。

#1.1文件系统工具
用法：# rospack find [包名称]
示例：$ rospack find roscpp
应输出：YOUR_INSTALL_PATH/share/roscpp
/opt/ros/groovy/share/roscpp

roscd # roscd [本地包名称[/子目录]]
示例：$ roscd roscpp

roscd log:使用roscd log可以切换到ROS保存日记文件的目录下。

rosls是rosbash命令集中的一部分，
它允许你直接按软件包的名称而不是绝对路径执行ls命令。
rosls [本地包名称[/子目录]]
rosls roscpp_tutorials:
cmake  package.xml  srv

#总结:
ROS命令工具的的命名方式：
rospack = ros + pack(age)
roscd = ros + cd
rosls = ros + ls