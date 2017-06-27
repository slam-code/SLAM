```c++

代码说明
下面我们将逐条解释代码，当然，之前解释过的代码就不再赘述了。
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
这是一个回调函数，当接收到 chatter 话题的时候就会被调用。消息是以 boost shared_ptr 指针的形式传输，这就意味着你可以存储它而又不需要复制数据。


ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

告诉 master 我们要订阅 chatter 话题上的消息。当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数。第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1000 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。

NodeHandle::subscribe() 返回 ros::Subscriber 对象,你必须让它处于活动状态直到你不再想订阅该消息。当这个对象销毁时，它将自动退订 chatter 话题的消息。

有各种不同的 NodeHandle::subscribe() 函数，允许你指定类的成员函数，甚至是 Boost.Function 对象可以调用的任何数据类型。roscpp overview 提供了更为详尽的信息。


ros::spin();
ros::spin() 进入自循环，可以尽可能快的调用消息回调函数。如果没有消息到达，它不会占用很多 CPU，所以不用担心。一旦 ros::ok() 返回 false，ros::spin() 就会立刻跳出自循环。这有可能是 ros::shutdown() 被调用，或者是用户按下了 Ctrl-C，使得 master 告诉节点要终止运行。也有可能是节点被人为关闭的。

还有其他的方法进行回调，但在这里我们不涉及。想要了解，可以参考 roscpp_tutorials package 里的一些 demo 应用。需要更为详尽的信息，可以参考 roscpp overview。

下边，我们来总结一下:

初始化ROS系统
订阅 chatter 话题
进入自循环，等待消息的到达
当消息到达，调用 chatterCallback() 函数

```

测试消息发布器和订阅器:

启动发布器
确保roscore可用，并运行：

$ roscore
catkin specific 如果使用catkin，确保你在调用catkin_make后，在运行你自己的程序前，已经source了catkin工作空间下的setup.sh文件：


# In your catkin workspace
$ cd ~/catkin_ws
$ source ./devel/setup.bash
In the last tutorial we made a publisher called "talker". Let's run it:

$ rosrun beginner_tutorials talker      (C++)
$ rosrun beginner_tutorials talker.py   (Python) 
你将看到如下的输出信息:

[INFO] [WallTime: 1314931831.774057] hello world 1314931831.77
[INFO] [WallTime: 1314931832.775497] hello world 1314931832.77 
[INFO] [WallTime: 1314931836.788106] hello world 1314931836.79
发布器节点已经启动运行。现在需要一个订阅器节点来接受发布的消息。

启动订阅器
上一教程，我们编写了一个名为"listener"的订阅器节点。现在运行它：

$ rosrun beginner_tutorials listener     (C++)
$ rosrun beginner_tutorials listener.py  (Python) 
你将会看到如下的输出信息:

[INFO] [WallTime: 1314931969.258941] /listener_17657_1314931968795I heard hello world 1314931969.26
[INFO] [WallTime: 1314931970.262246] /listener_17657_1314931968795I heard hello world 1314931970.26