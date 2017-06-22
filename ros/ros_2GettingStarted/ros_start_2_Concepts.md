ROS的概念分為三個層次：文件系统层、计算图层、社区层

1.ROS 文件系统层
文件系统层概念主要指在硬盘里能看到的ROS目录和文件, 例如：

Packages: Packages是在ROS中整理及組織軟體的主要單元。一個Packages包含節點（ROS runtime processes）、ROS程式庫（ROS-dependent library）、數據集（datasets）、配置文件（configuration files）⋯⋯等等。Packages是您在ROS中能建立及分享的最小單元。

Metapackages: Metapackages 是一组具体的服务相关的功能包。大部分的metpackages 只作为转换rosbuild Stacks的向后兼容的备选。

Package Manifests: Manifests (package.xml) 描述一个package的元信息，包括了package的名字，版本，功能简述，证书信息，依赖关系，以及一些其他的被export的package所有的信息。关于package.xml 的文件说明，参考REP-0127.

Repositories: 代码仓库是使用VCS版本控制系统的软件包集合，软件包利用版本控制维持同一版本，它能使用catkin自动发布工具bloom进行发布。这些代码仓库常通过映射来进行转换 rosbuild Stacks.仓库可以是只有一个软件包。

Message (msg) types: 存储在my_package/msg/MyMessageType.msg的Message文件，主要定义了ROS系统的messages传输的数据结构。

Service (srv) types: 存储在 my_package/srv/MyServiceType.srv的服务services文件，定义了ROS的服务通信时的请求（request ）和响应（response ）相关的数据结构。

2,ROS 计算图层
计算图是ROS在点对点网络里整合并处理数据的过程。基本计算图概念是 节点, 主机, 参数服务器, 消息, 服务, 话题, and 数据包，它们通过不同的方式提供数据给图层。

这些概念是在ros_comm库里实现的

Nodes: 节点主要执行计算处理 。ROS被设计为细粒度的模块化的系统;一个机器人控制系统通常有很多节点组成 。例如，一个节点控制激光测距仪，一个节点控制轮电机，一个节点执行定位，一个节点执行路径规划，一个节点提供系统图形界面，等等。一个ROS节点通过ROS客户端库 client library编写，例如 roscpp o或rospy

Master: 
Parameter Server: 
Messages: 
Topics: 
Services: 
Bags: 

3,社区层
The ROS Community Level concepts are ROS resources that enable separate communities to exchange software and knowledge. These resources include:

Distributions:
The ROS Wiki:
ROS Answers: 
Blog: 

4,Graph Resource Names

here are some example names:
/ (the global namespace)
/foo
/stanford/robot/name
/wg/node1