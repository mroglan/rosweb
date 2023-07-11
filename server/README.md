# RosWeb Server

## Dependencies

1. ROS 2. Has been tested with ROS 2 Humble, but should work with other ROS 2 distros.
2. Packages specified in `CMakeLists.txt`. 

## Starting the Server

From the `server` directory, run the following commands:

1. `source /opt/ros/{ROS_DISTRO}/setup.bash`
2. `mkdir build`
3. `cd build`
4. `cmake ..`
5. `make server`
6. `./server`