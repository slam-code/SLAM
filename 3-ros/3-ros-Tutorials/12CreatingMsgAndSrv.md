```c++
创建ROS消息和ROS服务
本教程详细介绍如何创建并编译ROS消息和服务，以及rosmsg, rossrv和roscp命令行工具的使用。
 

消息(msg)和服务(srv)介绍
消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。
服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。
msg文件存放在package的msg目录下，srv文件则存放在srv目录下。

msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下：

int8, int16, int32, int64 (plus uint*)
float32, float64
string
time, duration
other msg files
variable-length array[] and fixed-length array[C]
在ROS中有一个特殊的数据类型：Header，它含有时间戳和坐标系信息。在msg文件的第一行经常可以看到Header header的声明.

下面是一个msg文件的样例，它使用了Header，string，和其他另外两个消息类型。


  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
srv文件分为请求和响应两部分，由'---'分隔。下面是srv的一个样例：


int64 A
int64 B
---
int64 Sum
其中 A 和 B 是请求, 而Sum 是响应。

使用 msg
创建一个 msg
下面，我们将在之前创建的package里定义新的消息。



$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
上面是最简单的例子——在.msg文件中只有一行数据。当然，你可以仿造上面的形式多增加几行以得到更为复杂的消息：

string first_name
string last_name
uint8 age
uint32 score
接下来，还有关键的一步：我们要确保msg文件被转换成为C++，Python和其他语言的源代码：

查看package.xml, 确保它包含一下两条语句:

  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
如果没有，添加进去。 注意，在构建的时候，我们只需要"message_generation"。然而，在运行的时候，我们只需要"message_runtime"。

在你最喜爱的编辑器中打开CMakeLists.txt文件(可以参考前边的教程rosed).

在 CMakeLists.txt文件中，利用find_packag函数，增加对message_generation的依赖，这样就可以生成消息了。 你可以直接在COMPONENTS的列表里增加message_generation，就像这样：


# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
有时候你会发现，即使你没有调用find_package,你也可以编译通过。这是因为catkin把你所有的package都整合在一起，因此，如果其他的package调用了find_package，你的package的依赖就会是同样的配置。但是，在你单独编译时，忘记调用find_package会很容易出错。

同样，你需要确保你设置了运行依赖：

catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
找到如下代码块:

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
去掉注释符号#，用你的.msg文件替代Message*.msg，就像下边这样：


add_message_files(
  FILES
  Num.msg
)
手动添加.msg文件后，我们要确保CMake知道在什么时候重新配置我们的project。 确保添加了如下代码:

generate_messages()


现在，你可以生成自己的消息源代码了。如果你想立即实现，那么就跳过以下部分，到Common step for msg and srv.

使用 rosmsg
以上就是你创建消息的所有步骤。下面通过rosmsg show命令，检查ROS是否能够识消息。

使用方法:


$ rosmsg show [message type]
样例:


$ rosmsg show beginner_tutorials/Num
你将会看到:

int64 num
在上边的样例中,消息类型包含两部分：

beginner_tutorials -- 消息所在的package
Num -- 消息名Num.
如果你忘记了消息所在的package，你也可以省略掉package名。输入：


$ rosmsg show Num
你将会看到:

[beginner_tutorials/Num]:
int64 num
使用 srv
创建一个srv
在刚刚那个package中创建一个服务：


$ roscd beginner_tutorials
$ mkdir srv
这次我们不再手动创建服务，而是从其他的package中复制一个服务。 roscp是一个很实用的命令行工具，它实现了将文件从一个package复制到另外一个package的功能。

使用方法:


$ roscp [package_name] [file_to_copy_path] [copy_path]
现在我们可以从rospy_tutorials package中复制一个服务文件了：


$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
还有很关键的一步：我们要确保srv文件被转换成C++，Python和其他语言的源代码。


现在认为，你已经如前边所介绍的，在CMakeLists.txt文件中增加了对message_generation的依赖。:


# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
(对的, message_generation 对msg和srv都起作用)

同样，跟msg文件类似，你也需要在package.xml文件中做一些修改。查看上边的说明，增加额外的依赖项。

删掉#，去除对下边语句的注释:

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
用你自己的srv文件名替换掉那些Service*.srv文件:


add_service_files(
  FILES
  AddTwoInts.srv
)


现在，你可以生成自己的服务源代码了。如果你想立即实现，那么就跳过以下部分，到Common step for msg and srv.

使用 rossrv
以上就是创建一个服务所需的所有步骤。下面通过rosmsg show命令，检查ROS是否能够识该服务。

使用方法:


$ rossrv show <service type>
例子:


$ rossrv show beginner_tutorials/AddTwoInts
你将会看到:

int64 a
int64 b
---
int64 sum
跟rosmsg类似, 你也可以不指定具体的package名来查找服务文件：


$ rossrv show AddTwoInts
[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum

[rospy_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
msg和srv都需要的步骤

接下来，在CMakeLists.txt中找到如下部分:

# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
去掉注释并附加上所有你消息文件所依赖的那些含有.msg文件的package（这个例子是依赖std_msgs,不要添加roscpp,rospy)，结果如下:


generate_messages(
  DEPENDENCIES
  std_msgs
)
由于增加了新的消息，所以我们需要重新编译我们的package：


# In your catkin workspace
$ cd ../..
$ catkin_make
$ cd -

所有在msg路径下的.msg文件都将转换为ROS所支持语言的源代码。生成的C++头文件将会放置在~/catkin_ws/devel/include/beginner_tutorials/。 Python脚本语言会在 ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg 目录下创建。 lisp文件会出现在 ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/ 路径下.

详尽的消息格式请参考Message Description Language 页面.

获得帮助
我们已经接触到不少的ROS工具了。有时候很难记住他们所需要的参数。还好大多数ROS工具都提供了帮助。

输入:


$ rosmsg -h
你可以看到一系列的rosmsg子命令.
Commands:
  rosmsg show Show message description
  rosmsg users  Find files that use message
  rosmsg md5  Display message md5sum
  rosmsg package  List messages in a package
  rosmsg packages List packages that contain messages
同样你也可以获得子命令的帮助：


$ rosmsg show -h
这会现实rosmsg show 所需的参数：
Usage: rosmsg show [options] <message type>

Options:
  -h, --help  show this help message and exit
  -r, --raw   show raw message text, including comments
回顾
总结一下到目前为止我们接触过的一些命令：

rospack = ros+pack(age) : provides information related to ROS packages
rosstack = ros+stack : provides information related to ROS stacks
roscd = ros+cd : changes directory to a ROS package or stack
rosls = ros+ls : lists files in a ROS package
roscp = ros+cp : copies files from/to a ROS package
rosmsg = ros+msg : provides information related to ROS message definitions
rossrv = ros+srv : provides information related to ROS service definitions
rosmake = ros+make : makes (compiles) a ROS package


```