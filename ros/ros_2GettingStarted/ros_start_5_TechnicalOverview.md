#1,概述
This technical overview goes into greater detail about the implementation of ROS. Most ROS users do not need to know these details, but they are important for those wishing to write their own ROS client libraries or those wishing to integrate other systems with ROS.

#2,Master,主节点,管理所有的node,使用xml的rpc通信.

The Master is implemented via XMLRPC, which is a stateless, HTTP-based protocol. XMLRPC was chosen primarily because it is relatively lightweight, does not require a stateful connection, and has wide availability in a variety of programming languages. For example, in Python, you can start any Python interpreter and begin interacting with the ROS Master:

$ python
>>> from xmlrpclib import ServerProxy
>>> import os
>>> master = ServerProxy(os.environ['ROS_MASTER_URI'])
>>> master.getSystemState('/')
[1, 'current system state', [[['/rosout_agg', ['/rosout']]], [['/time', ['/rosout']], ['/rosout', ['/rosout']], ['/clock', ['/rosout']]], [['/rosout/set_logger_level', ['/rosout']], ['/rosout/get_loggers', ['/rosout']]]]]

The Master has registration APIs, which allow nodes to register as publishers, subscribers, and service providers. The Master has a URI and is stored in the ROS_MASTER_URI environment variable. This URI corresponds to the host:port of the XML-RPC server it is running. By default, the Master will bind to port 11311.

For more information, including an API listing, please see Master API.http://wiki.ros.org/ROS/Master_API

#3,Parameter Server

A ROS node has several APIs

1,A slave API. The slave API is an XMLRPC API that has two roles: receiving callbacks from the Master, and negotiating connections with other nodes. For a detailed API listing, please see Slave API.
2,A topic transport protocol implementation (see TCPROS and UDPROS). Nodes establish topic connections with each other using an agreed protocol. The most general protocol is TCPROS, which uses persistent, stateful TCP/IP socket connections.
3,A command-line API. Every node should support command-line remapping arguments, which enable names within a node to be configured at runtime.

#4,Topic Transports 消息的转发

主要是使用tcp,其次是udp

There are many ways to ship data around a network, and each has advantages and disadvantages, depending largely on the application. TCP is widely used because it provides a simple, reliable communication stream. TCP packets always arrive in order, and lost packets are resent until they arrive. While great for wired Ethernet networks, these features become bugs when the underlying network is a lossy WiFi or cell modem connection. In this situation, UDP is more appropriate. When multiple subscribers are grouped on a single subnet, it may be most efficient for the publisher to communicate with all of them simultaneously via UDP broadcast.

for these reasons, ROS does not commit to a single transport. Given a publisher URI, a subscribing node negotiates a connection, using the appropriate transport, with that publisher, via XMLRPC. The result of the negotiation is that the two nodes are connected, with messages streaming from publisher to subscriber.

Each transport has its own protocol for how the message data is exchanged.

#5,Message serialization and msg MD5 sums 消息的序列化 和MD5校验
Messages are serialized in a very compact representation that roughly corresponds to a c-struct-like serialization of the message data in little endian format. The compact representation means that two nodes communicating must agree on the layout of the message data.

Message types (msgs) in ROS are versioned using a special MD5 sum calculation of the msg text. In general, client libraries do not implement this MD5 sum calculation directly, instead storing this MD5 sum in auto-generated message source code using the output of roslib/scripts/gendeps. 

#6,Establishing a topic connection 建立topic连接

Putting it all together, the sequence by which two nodes begin exchanging messages is:

Subscriber starts. It reads its command-line remapping arguments to resolve which topic name it will use. (Remapping Arguments)
Publisher starts. It reads its command-line remapping arguments to resolve which topic name it will use. (Remapping Arguments)
Subscriber registers with the Master. (XMLRPC)
Publisher registers with the Master. (XMLRPC)
Master informs Subscriber of new Publisher. (XMLRPC)
Subscriber contacts Publisher to request a topic connection and negotiate the transport protocol. (XMLRPC)
Publisher sends Subscriber the settings for the selected transport protocol. (XMLRPC)
Subscriber connects to Publisher using the selected transport protocol. (TCPROS, etc...)