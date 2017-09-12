
运行小乌龟:
roscore
rosrun turtlesim turtlesim_node        #turlesim_node属于turtlesim软件包
rosrun turtlesim  turtle_teleop_key   #turtle_teleop_key属于turtlesim软件包

查看已安装ros软件包
rospack list

每个程序包由一个清单文件组成 package.xml
如/opt/ros/kinetic/share/tf2/package.xml
包含package.xml的目录就是该软件包的目录
可执行文件放在lib目录下


rospack find turtle_tf #查找包
    /opt/ros/kinetic/share/turtle_tf

rosls turtle_tf -al    # 等同于 ls -al 
    drwxr-xr-x   2 root root  4096 6月   7 16:29 cmake
    drwxr-xr-x   2 root root  4096 6月   7 16:29 launch
    -rw-r--r--   1 root root  1207 4月   8  2016 package.xml
    drwxr-xr-x   2 root root  4096 6月   7 16:29 rviz

roscd turtle_tf        # cd到该包的目录下
rosls turtlesim        # ls


#每一个包都是一个文件夹
#节点node相对独立,为了实现节点间的相互通信,需使用节点管理器 The Master
roscore #启动节点管理器.节点管理器生命期是全部程序的运行期
roslaunch #启动多个节点


ros的一个程序实例被称为一个节点node

rosrun turtlesim  turtle_teleop_key  #启动一个实例,包名是turtlesim,可执行文件是turtle_teleop_key

rosnode list  #查看运行节点

rosnode info turtlesim #查看节点info


#话题和消息 ,节点与节点通信

rqt_graph  #查看计算图,展示了节点的订阅关系

rostopic list #获取活跃列表

rostopic echo /turtle1/cmd_vel  #查看cmd发布的消息(键盘输入的消息),'---'表示一条消息

rostopic hz  /turtle1/cmd_vel  #查看消息发布频率

rostopic bw  /turtle1/cmd_vel  #查看消息占用带宽, 106.76B/s

rostopic info  /turtle1/cmd_vel #获取话题info
        Type: geometry_msgs/Twist                   #消息类型,type
        Publishers: 
         * /teleop_turtle (http://yang:39986/) #订阅者
        Subscribers: 
         * /turtlesim (http://yang:42248/)


rosmsg show turtlesim/Color  #查看消息type
                uint8 r
                uint8 g
                uint8 b


rosmsg show turtlesim/Pose  #type,以field列表形式.
                float32 x
                float32 y
                float32 theta
                float32 linear_velocity
                float32 angular_velocity

rosmsg show geometry_msgs/Twist  # 复合域,有2个子域
                geometry_msgs/Vector3 linear
                  float64 x
                  float64 y
                  float64 z
                geometry_msgs/Vector3 angular
                  float64 x
                  float64 y
                  float64 z

#手动发布消息message,1hz发布,2hz发布

 rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist '[2,0,0]' '[0,0,0]' #2m/s 直线前进

 rostopic pub -r 2 /turtle1/cmd_vel geometry_msgs/Twist '[2,0,0]' '[0,0,4]'   #2m/s 绕z轴旋转,垂直于电脑屏幕
  2维仿真只关注与 liner.x 和angular.z,其余变化不起作用


#理解消息类型

package-name/type-name:
如geometry_msgs/Vector3 

#消息是多对多的
 
 一个发布者只管发布,可以多个订阅

 "生产"message的程序如(turtle_teleop_key)只管发布消息,不管消息如何被消费
 "消费"message的程序如(turlesim_node) 只管订阅该topic,而不管message如何被生产

 一对一通信是 services (少用)

#roswtf 全面检测系统








