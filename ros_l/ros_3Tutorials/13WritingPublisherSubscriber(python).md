``` python


写一个简单的消息发布器和订阅器 (Python)
Description: 本教程将通过Python编写一个发布器节点和订阅器节点。


Writing the Publisher Node
"Node" is the ROS term for an executable that is connected to the ROS network. Here we'll create the publisher ("talker") node which will continually broadcast a message.

Change directory into the beginner_tutorials package, you created in the earlier tutorial, creating a package:

$ roscd beginner_tutorials

$ mkdir scripts
$ cd scripts
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py

$ chmod +x talker.py

```


```python


import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

```

```python

注释:
# 1
import rospy
from std_msgs.msg import String
you need to import rospy if you are writing a ROS Node. The std_msgs.msg import is so that we can reuse the std_msgs/String message type (a simple string container) for publishing.

# 2
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

# 3
     rate = rospy.Rate(10) # 10hz


# 4

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

```

http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28python%29