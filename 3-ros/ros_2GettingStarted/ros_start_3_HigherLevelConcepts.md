The core ROS platform attempts to be as architecture-agnostic as possible. It provides several different modes of communicating data (topics, services, Parameter Server), but it doesn't prescribe how they're used or how they are named. This approach allows ROS to be easily integrated with a variety of architectures, but higher-level concepts are necessary for building larger systems on top of ROS.

There are several stacks, such as common, common_msgs, and geometry, that provide these higher-level concepts for use with ROS and are described below.

ROS尽量做到与计算架构无关,并提供了几个通信方式,但是没有描述如何使用.高级概念有助于建立大型系统.以下:
#1,Coordinate Frames/Transforms
坐标系变换

tf和tf2包提供了不同坐标系的变换.如世界坐标->传感坐标

#2,Actions/Tasks 任务

The actionlib package defines a common, topic-based interface for preemptible tasks in ROS.
actionlib提供了基于topic的抢占式任务接口

#3,Message Ontology 消息体

The common_msgs stack provide a base message ontology for robotic systems. It defines several classes of messages, including:

actionlib_msgs: messages for representing actions.
diagnostic_msgs: messages for sending diagnostic data.
geometry_msgs: messages for representing common geometric primitives.
nav_msgs: messages for navigation.
sensor_msgs: messages for representing sensor data.

#4,Plugins,插件
pluginlib provides a library for dynamically loading libraries in C++ code.

#5,Filters,滤波器
The filters package provides a C++ library for processing data using a sequence of filters.

#6,Robot Model 机器人模型,用urdf语言描述
The urdf package defines an XML format for representing a robot model and provides a C++ parser.
 