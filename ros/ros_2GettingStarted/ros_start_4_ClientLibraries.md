#1,客户端函数库
A ROS client library is a collection of code that eases the job of the ROS programmer. It takes many of the ROS concepts and makes them accessible via code. In general, these libraries let you write ROS nodes, publish and subscribe to topics, write and call services, and use the Parameter Server. Such a library can be implemented in any programming language, though the current focus is on providing robust C++ and Python support.
目前主要支持c++和 Python

#2,Main client libraries
C++客户端:roscpp

roscpp :   designed to be the high performance library for ROS.

Python客户端
rospy: rospy is the pure Python client library for ROS and is designed to provide the advantages of an object-oriented scripting language to ROS. The design of rospy favors implementation speed (i.e. developer time) over runtime performance so that algorithms can be quickly prototyped and tested within ROS. It is also ideal for non-critical-path code, such as configuration and initialization code. Many of the ROS tools are written in rospy to take advantage of the type introspection capabilities. The ROS Master, roslaunch, and other ros tools are developed in rospy, so Python is a core dependency of ROS.

LISP语言:
roslisp: roslisp is a client library for LISP and is currently being used for the development of planning libraries. It supports both standalone node creation and interactive use in a running ROS system.

其他实验性质的语言客户端包括

rosjava: rosjava is an implementation of ROS in pure-Java with Android support.
roslua: roslua is a client library for Lua, which is a light-weight yet powerful, embeddable scripting language. The client library is currently in an experimental and active development stage.
roscs: roscs is a client library for Mono/.NET. It can be used by any Mono/.NET language, including C#, Iron Python, Iron Ruby, etc. The ROS build system will create .DLL and .so files for each package written in roscs.
roseus: roseus is a client library for EusLisp language.
PhaROS is an client library under MIT Licence for the Pharo free Smalltalk language.
rosR: rosR is an ros language extension for the statistical programming language R.