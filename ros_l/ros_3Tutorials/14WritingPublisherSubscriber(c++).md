```c++

编写简单的消息发布器和订阅器 (C++)
编写发布器节点
『节点』(Node) 是指 ROS 网络中可执行文件。接下来，我们将会创建一个发布器节点("talker")，它将不断的在 ROS 网络中广播消息。
切换到之前创建的 beginner_tutorials package 路径下：
cd ~/catkin_ws/src/beginner_tutorials
在 beginner_tutorials package 路径下创建一个src文件夹：
mkdir -p ~/catkin_ws/src/beginner_tutorials/src
这个文件夹将会用来放置 beginner_tutorials package 的所有源代码。
在 beginner_tutorials package 里创建 src/talker.cpp 文件，并将如下代码粘贴到文件内：
```

```c++

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

```


```c++

# 代码说明
现在，我们来分段解释代码。
#include "ros/ros.h"
ros/ros.h 是一个实用的头文件，它引用了 ROS 系统中大部分常用的头文件。

#include "std_msgs/String.h"
引用了 std_msgs/String 消息, 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件。需要关于消息的定义，可以参考 msg 页面。

ros::init(argc, argv, "talker");
初始化 ROS 。它允许 ROS 通过命令行进行名称重映射——然而这并不是现在讨论的重点。在这里，我们也可以指定节点的名称——运行过程中，节点的名称必须唯一。这里的名称必须是一个 base name ，也就是说，
名称内不能包含 / 等符号。


ros::NodeHandle n;
为这个进程的节点创建一个句柄。第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源。

ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息。这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。第二个参数是发布序列的大小。如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。

NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用： 1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 2) 如果消息类型不对,它会拒绝发布。


ros::Rate loop_rate(10);
ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。在这个例子中，我们让它以 10Hz 的频率运行。


int count = 0;
  while (ros::ok())
  {

roscpp 会默认生成一个 SIGINT 句柄，它负责处理 Ctrl-C 键盘操作——使得 ros::ok() 返回 false。
如果下列条件之一发生，ros::ok() 返回false：
SIGINT 被触发 (Ctrl-C)
被另一同名节点踢出 ROS 网络
ros::shutdown() 被程序的另一部分调用
节点中的所有 ros::NodeHandles 都已经被销毁
一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效.

std_msgs::String msg;

std::stringstream ss;
ss << "hello world " << count;
msg.data = ss.str();
我们使用一个由 msg file 文件产生的『消息自适应』类在 ROS 网络中广播消息。
现在我们使用标准的String消息，它只有一个数据成员 "data"。当然，你也可以发布更复杂的消息类型。

chatter_pub.publish(msg);
我们向所有订阅 chatter 话题的节点发送消息。


ROS_INFO("%s", msg.data.c_str());
用来代替 printf/cout 等函数。

 ros::spinOnce();
在这个例子中并不是一定要调用 ros::spinOnce()，因为我们不接受回调。然而，如果你的程序里包含其他回调函数，最好在这里加上 ros::spinOnce()这一语句，否则你的回调函数就永远也不会被调用了。


loop_rate.sleep();
这条语句是调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz。

对上边的内容进行一下总结：
初始化 ROS 系统,
在 ROS 网络内广播我们将要在 chatter 话题上发布 std_msgs/String 类型的消息,
以每秒 10 次的频率在 chatter 上发布消息,
接下来我们要编写一个节点来接收这个消息。


```

