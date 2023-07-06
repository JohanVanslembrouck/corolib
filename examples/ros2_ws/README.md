# Using ROS 2 (humble) with corolib

## Contents 

src/demo_nodes_cpp_co is based upon https://github.com/ros2/demos/tree/humble/demo_nodes_cpp.
The name demo_nodes_cpp_co reflects the addition of coroutine examples to demo_nodes_cpp and avoids name clashes because demo_nodes_cpp is part of the ROS 2 installation.

src/action_tutorials_cpp_co is based upon the description in https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html. The use of action_tutorials_cpp_co instead of action_tutorials_cpp also avoids name clashes.

It has been created with the following command:

    ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp_co

## Build and run instructions

The following instructions are for Linux (Ubuntu 22.04 LTS).

    colcon build

Enter in every terminal that you open:

    source install/local_setup.bash

## Running the applications

### demo_nodes_cpp_co

In one terminal:

    ros2 run demo_nodes_cpp_co add_two_ints_server

In a second terminal:

    ros2 run demo_nodes_cpp_co add_two_ints_client_co

Alternatively:

    ros2 run demo_nodes_cpp_co add_two_ints_client_async_co

### action_tutorials_cpp_co

In one terminal:

    ros2 run action_tutorials_cpp_co fibonacci_action_server

In a second terminal:

    ros2 run action_tutorials_cpp_co fibonacci_action_client_co
