理解ROS服务和参数:

本教程假设从前一教程启动的turtlesim_node仍在运行，现在我们来看看turtlesim提供了什么服务：

ROS Services
服务（services）是节点之间通讯的另一种方式。服务允许节点发送请求（request） 并获得一个响应（response）

#1,使用rosservice
rosservice可以很轻松的使用 ROS 客户端/服务器框架提供的服务。rosservice提供了很多可以在topic上使用的命令，如下所示：

使用方法:

rosservice list         输出可用服务的信息
rosservice call         调用带参数的服务
rosservice type         输出服务类型
rosservice find         依据类型寻找服务find services by service type
rosservice uri          输出服务的ROSRPC uri
rosservice list

先初始,运行小乌龟:
roscore
rosrun turtlesim turtlesim_node        #turlesim_node属于turtlesim软件包
rosrun turtlesim  turtle_teleop_key   #turtle_teleop_key属于turtlesim软件包

$ rosservice list
list 命令显示turtlesim节点提供了9个服务：重置（reset）, 清除（clear）, 再生（spawn）, 终止（kill）, turtle1/set_pen, /turtle1/teleport_absolute, /turtle1/teleport_relative, turtlesim/get_loggers, and turtlesim/set_logger_level. 同时还有另外两个rosout节点提供的服务: /rosout/get_loggers and /rosout/set_logger_level.

/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
我们使用rosservice type命令更进一步查看clear服务:

rosservice type

使用方法:

rosservice type [service]
我们来看看clear服务的类型:

$ rosservice type clear

std_srvs/Empty
服务的类型为空（empty),这表明在调用这个服务是不需要参数（比如，请求不需要发送数据，响应也没有数据）。下面我们使用rosservice call命令调用服务：

rosservice call

使用方法:

rosservice call [service] [args]
因为服务类型是空，所以进行无参数调用：

$ rosservice call clear
正如我们所期待的，服务清除了turtlesim_node的背景上的轨迹。



通过查看再生（spawn）服务的信息，我们来了解带参数的服务:

$ rosservice type spawn| rossrv show
float32 x
float32 y
float32 theta
string name
---
string name
这个服务使得我们可以在给定的位置和角度生成一只新的乌龟。名字参数是可选的，这里我们不设具体的名字，让turtlesim自动创建一个。

$ rosservice call spawn 2 2 0.2 ""
服务返回了新产生的乌龟的名字：

name: turtle2



#2,Using rosparam
rosparam使得我们能够存储并操作ROS 参数服务器（Parameter Server）上的数据。参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型。rosparam使用YAML标记语言的语法。一般而言，YAML的表述很自然：1 是整型, 1.0 是浮点型, one是字符串, true是布尔, [1, 2, 3]是整型列表, {a: b, c: d}是字典. rosparam有很多指令可以用来操作参数，如下所示:

使用方法:

rosparam set            设置参数
rosparam get            获取参数
rosparam load           从文件读取参数
rosparam dump           向文件中写入参数
rosparam delete         删除参数
rosparam list           列出参数名
我们来看看现在参数服务器上都有哪些参数：

rosparam list


$ rosparam list
我们可以看到turtlesim节点在参数服务器上有3个参数用于设定背景颜色：

/background_b
/background_g
/background_r
/roslaunch/uris/aqy:51932
/run_id
Let's change one of the parameter values using rosparam set:

rosparam set and rosparam get

Usage:
rosparam set [param_name]
rosparam get [param_name]
查看:
rosparam get background_r
110
现在我们修改背景颜色的红色通道：
$ rosparam set background_r 150
上述指令修改了参数的值，现在我们调用清除服务使得修改后的参数生效：
$ rosservice call clear (使之生效)

我们可以使用rosparam get /来显示参数服务器上的所有内容：

$ rosparam get /

        background_b: 255
        background_g: 86
        background_r: 1
        rosdistro: 'kinetic

          '
        roslaunch:
          uris: {host_yang__42939: 'http://yang:42939/'}
        rosversion: '1.12.7

          '
        run_id: 52e95574-574b-11e7-8833-60a44c2c889c


你可能希望存储这些信息以备今后重新读取。这通过rosparam很容易就可以实现:

rosparam dump and rosparam load

使用方法:

rosparam dump [file_name]
rosparam load [file_name] [namespace]
现在我们将所有的参数写入params.yaml文件：

$ rosparam dump params.yaml
你甚至可以将yaml文件重载入新的命名空间，比如说copy空间:

$ rosparam load params.yaml copy
$ rosparam get copy/background_b
255

