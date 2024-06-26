# corolib examples

This directory/folder has the following subdirectories/subfolders that use an asynchronous communication framework:

* boost
* cppcoro
* grpc
* libevent
* qt5
* ros2_ws (ROS2 Humble)
* tao

See the README.md file in those directories for further information.

The following subdirectories do not use any third-party communication framework, apart from some examples in corolab that use Boost ASIO.

* tutorial contains standalone examples illustrating the use of corolib.
* why-coroutines contains examples that explain the advantages of C++ coroutines for writing (distributed) asynchronous applications.
* corolab contains examples that do not use the corolib include files or libraries, but were the basis from where corolib has been "distilled."
