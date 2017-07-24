理解 ROS节点:介绍 ROS 图（graph）概念 并讨论roscore、rosnode和 rosrun 命令行工具的使用。

安装轻量级的模拟器:
$ sudo apt-get install ros-<distro>-ros-tutorials
用你使用的ROS发行版本名称(例如electric、fuerte、groovy、hydro等)替换掉'<distro>'。

#1,图概念概述
        Nodes:节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。
        Messages:消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。
        Topics:话题,节点可以发布消息到话题，也可以订阅话题以接收消息。
        Master:节点管理器，ROS名称服务 (比如帮助节点找到彼此)。
        rosout: ROS中相当于stdout/stderr。
        roscore: 主机+ rosout + 参数服务器 (参数服务器会在后面介绍)

#2,节点
        一个节点其实只不过是ROS程序包中的一个可执行文件。ROS节点可以使用ROS客户库与其他节点
        通信。节点可以发布或接收一个话题。节点也可以提供或使用某种服务。

#3,客户端库
        ROS客户端库允许使用不同编程语言编写的节点之间互相通信:

        rospy = python 客户端库
        roscpp = c++ 客户端库
        roscore
        roscore 是你在运行所有ROS程序前首先要运行的命令。
        请运行:
        $ roscore

#4,rosnode list 列出活跃的节点:
        /rosout 表示当前只有一个节点在运行: rosout。因为这个节点用于收集和记录节点调
    rosnode info 命令返回的是关于一个特定节点的信息。
    $ rosnode info /rosout

                Node [/rosout]
                Publications: 
                 * /rosout_agg [rosgraph_msgs/Log]
                Subscriptions: 
                 * /rosout [unknown type]
                Services: 
                 * /rosout/set_logger_level
                 * /rosout/get_loggers
                contacting node http://shen-yang:36009/ ...
                Pid: 13726

#5,使用 rosrun
        rosrun 允许你使用包名直接运行一个包内的节点(而不需要知道这个包的路径)。
        用法:rosrun [package_name] [node_name]
        $ rosrun turtlesim turtlesim_node

#6,rosnode list
        你会看见类似于:
        /rosout
        /turtlesim

#7,使用Remapping Argument改变节点名称:
        $ rosrun turtlesim turtlesim_node __name:=my_turtle
        现在，我们退回使用 rosnode list:

        $ rosnode list
        你会看见类似于:
        /rosout
        /my_turtle

#8,我们可以看到新的/my_turtle 节点。使用另外一个 rosnode 指令, ping, 来测试:
        $ rosnode ping my_turtle
        rosnode: node is [/my_turtle]
        pinging /my_turtle with a timeout of 3.0s
        xmlrpc reply from http://aqy:42235/     time=1.152992ms
        xmlrpc reply from http://aqy:42235/     time=1.120090ms
        xmlrpc reply from http://aqy:42235/     time=1.700878ms
        xmlrpc reply from http://aqy:42235/     time=1.127958ms
#9,总结
        roscore = ros+core : master (provides name service for ROS) + rosout
        (stdout/stderr) + parameter server (parameter server will be introduced

        later)

        rosnode = ros+node : ROS tool to get information about a node.
        rosrun = ros+run : runs a node from a given package.
