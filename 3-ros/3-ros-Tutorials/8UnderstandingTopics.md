##理解ROS话题:
        本教程介绍ROS话题（topics）以及如何使用rostopic 和 rxplot 命令行工具。


#1,运行example:
        roscore
        rosrun turtlesim turtlesim_node
        rosrun turtlesim turtle_teleop_key
        使用方向键控制小乌龟移动

#2,ROS Topics
        urtlesim_node节点和turtle_teleop_key节点之间是通过一个ROS话题来互相通信的。
        turtle_teleop_key在一个话题上发布按键输入消息，而turtlesim则订阅该话题以接收该消息。
        下面让我们使用rqt_graph来显示当前运行的节点和话题。



#3,rqt_graph
        rqt_graph能够创建一个显示当前系统运行情况的动态图形。
        rqt_graph是rqt程序包中的一部分。如果你没有安装，请通过以下命令来安装：
        $ sudo apt-get install ros-<distro>-rqt
        $ sudo apt-get install ros-<distro>-rqt-common-plugins
        请使用你的ROS版本名称（比如fuerte、groovy、hydro等）来替换掉<distro>。
        在一个新终端中运行:
        $ rosrun rqt_graph rqt_graph

        turtlesim_node和turtle_teleop_key节点正通过一个名为 /turtle1/command_velocity的话题来互相通信。

#4,rostopic介绍
        rostopic命令工具能让你获取有关ROS话题的信息。
        你可以使用帮助选项查看rostopic的子命令：
        $ rostopic -h
        rostopic bw     display bandwidth used by topic
        rostopic echo   print messages to screen
        rostopic hz     display publishing rate of topic
        rostopic list   print information about active topics
        rostopic pub    publish data to topic
        rostopic type   print topic type

#5,rostopic echo

        rostopic echo可以显示在某个话题上发布的数据。
        用法：rostopic echo [topic]
        让我们在一个新终端中看一下turtle_teleop_key节点在/turtle1/cmd_vel下发布的数据
        $ rostopic echo /turtle1/cmd_vel
        通过按下方向键使turtle_teleop_key节点发布数据。
         现在当你按下向上方向键时应该会看到下面的信息：

        ---
        linear: 2.0
        angular: 0.0
        ---
        linear: 2.0
        angular: 0.0
        再看一下rqt_graph（你可能需要刷新一下ROS graph）。正如你所看到的，rostopic echo(红色显示部分）现在也订阅了turtle1/cmd_vel话题。
 
 #6,rostopic list 能够列出所有当前订阅和发布的话题

        /rosout
        /rosout_agg
        /turtle1/cmd_ve

##ROS Messages
    话题之间的通信是通过在节点之间发送ROS消息实现的。对于发布器(turtle_teleop_key)和订阅器(turtulesim_node)之间的通信，发布器和订阅器之间必须发送和接收相同类型的消息。这意味着话题的类型是由发布在它上面的消息类型决定的。使用rostopic type命令可以查看
    发布在某个话题上的消息类型。
#7,使用 rostopic type
rostopic type 命令用来查看所发布话题的消息类型。
用法：
rostopic type [topic]
$ rostopic type /turtle1/cmd_vel
     消息类型,你应该会看到:
     geometry_msgs/Twist
消息的详细情况:
$ rosmsg show geometry_msgs/Twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
现在我们已经知道了turtlesim节点所期望的消息类型，接下来我们就可以给turtle发布命令了。

#8,学习 rostopic
    现在我们已经了解了什么是ROS的消息，接下来我们开始结合消息来使用rostopic。
    #8.1,rostopic pub可以把数据发布到当前某个正在广播的话题上。
    用法：
    rostopic pub [topic] [msg_type] [args]
    示例：
    $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    以上命令会发送一条消息给turtlesim，告诉它以2.0大小的线速度和1.8大小的角速度开始移动。
#8.2,解析:
    rostopic pub :这条命令将会发布消息到某个给定的话题。
    -1（单个破折号）:这个参数选项使rostopic发布一条消息后马上退出。
    /turtle1/cmd_vel:这是消息所发布到的话题名称
    geometry_msgs/Twist :这是所发布消息的类型。
    --:参数部分
    频率为1Hz的命令流来保持移动状态。:
    rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
#9,使用 rostopic hz

rostopic hz命令可以用来查看数据发布的频率。
    用法：
    rostopic hz [topic]
    我们看一下turtlesim_node发布/turtle/pose时有多快：
    $ rostopic hz /turtle1/pose
        subscribed to [/turtle1/pose]
        average rate: 59.354
                min: 0.005s max: 0.027s std dev: 0.00284s window: 58
        average rate: 59.459
                min: 0.005s max: 0.027s std dev: 0.00271s window: 118
        average rate: 59.539
                min: 0.004s max: 0.030s std dev: 0.00339s window: 177
        average rate: 59.492
                min: 0.004s max: 0.030s std dev: 0.00380s window: 237
        average rate: 59.463
                min: 0.004s max: 0.030s std dev: 0.00380s window: 290
    在我们可以知道了turtlesim正以大约60Hz的频率发布数据给turtle。我们也可以结合rostopic type和rosmsg show命令来获取关于某个话题的更深层次的信息:
    rostopic type /turtle1/cmd_vel | rosmsg show
#10,使用 rqt_plot
    rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形。这里我们将使用rqt_plot命令来绘制正在发布到/turtle1/pose话题上的数据变化图形。首先，在一个新终端中运行rqt_plot命令：
    
    $ rosrun rqt_plot rqt_plot
    这会弹出一个新窗口，在窗口左上角的一个文本框里面你可以添加需要绘制的话题。在里面输入/turtle1/pose/x后之前处于禁用状态的加号按钮将会被使能变亮。按一下该按钮，并对/turtle1/pose/y重复相同的过程。现在你会在图形中看到turtle的x-y位置坐标图。