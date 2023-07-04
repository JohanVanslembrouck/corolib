# Using ROS 2 (humble) with corolib

## Contents 

src/demo_nodes_cpp_co is based upon https://github.com/ros2/demos/tree/humble/demo_nodes_cpp.
The name has been adapted to reflect the addition of coroutine examples and to avoid name clashes becomes demo_nodes_cpp is part of the ROS 2 installation.

## Build and run instructions

The following instructions are for Linux (Ubuntu 22.04 LTS).

In one terminal:

* colcon build
* source install/local_setup.bash
* ros2 run demo_nodes_cpp_co add_two_ints_server

In a second terminal:

* source install/local_setup.bash
* ros2 run demo_nodes_cpp_co add_two_ints_client_co
