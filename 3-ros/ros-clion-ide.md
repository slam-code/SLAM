


https://stackoverflow.com/questions/33172132/setting-up-ros-package-in-clion

To add on to what WillC suggested, you can also 
modify the desktop entry to start the application from bash instead of manually doing so.

To do this, edit the desktop file located at

my执行:
vim ~/.local/share/applications/jetbrains-clion.desktop
by modifying the line containing Exec= to

shen执行:Exec=bash -i -c "/INSTALL_LOCATION/clion-2016.3.2/bin/clion.sh" %f


配置ROS+Clion
ROS WIKI 提供了几种IDE的配置方法，里面没有Clion的，但是观察一下会发现都大同小异。
只是在启动文件里面加一些参数，使启动之后可以识别你的ROS系统。 

在/usr/share/applications中修改jetbrains-clion.desktop文件，
修改
Exec="/home/clion-2016.2/bin/clion.sh" %f
为
Exec=bash -i -c "/home/clion-2016.2/bin/clion.sh" %f
即添加 bash -i -c

这样即可，编辑好源文件后。修改CMakeLists.txt，添加package依赖项
例：find_package(catkin REQUIRED roscpp)
target_link_libraries(first ${catkin_LIRRARIES}) //链接库文件
这样就可以进行编译了。
