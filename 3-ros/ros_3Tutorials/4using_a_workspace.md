Building and using catkin packages in a workspace

1,With catkin_make

$ cd ~/catkin_ws/src/beginner_tutorials/src

# Add/Edit source files

$ cd ~/catkin_ws/src/beginner_tutorials

# Update CMakeFiles.txt to reflect any changes to your sources

$ cd ~/catkin_ws

$ catkin_make -DCMAKE_BUILD_TYPE=Release

$ cd ~/catkin_ws
$ catkin_make install

# This is an example
$ cd ~/catkin_ws
$ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/groovy  # might need sudo

catkin_make -DCMAKE_VERBOSE_MAKEFILE=ON