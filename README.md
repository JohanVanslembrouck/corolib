# corolib

A C++ coroutine library for writing asynchronous distributed applications.

## Installation and building

For its communication, corolib currently uses any of the following frameworks:

* Boost ASIO library
* Qt (QTcpSocket, QTcpServer)
* gRPC
* ROS 2 (Humble)

Other publicly available asynchronous communication frameworks may follow in the future.

These frameworks have to be installed on your computer before building corolib from the root CMakeLists.txt file.
The installation steps are described below.

However, without any of these libraries, there are still a large number of examples illustrating the use of corolib.
None of them will involve communication, i.e. they are all stand-alone examples.

### On Windows 10 or 11

#### Boost

I originally used Boost 1.70 (now Boost 1.82), but many other versions will do.

Starting point: https://www.boost.org/

Download release 1.82 from https://boostorg.jfrog.io/artifactory/main/release/1.82.0/source/

I copied boost_1_82_0.zip to C:\local\boost and unzipped it into this directory.

In a Windows terminal (e.g. Command Prompt or PowerShell), go to C:\local\boost\boost_1_82_0

Follow the instructions in https://www.boost.org/doc/libs/1_82_0/more/getting_started/windows.html#simplified-build-from-source

Step 1: .\bootstrap.bat

This will generate b2.exe

Step 2: .\b2.exe

b2.exe generates the libraries we will need. Its execution takes a while.

The root directory for the include files is C:/local/boost/boost_1_82_0.

The root directory for the libraries is C:/local/boost/boost_1_82_0/stage/lib.

#### Qt

Starting points: https://www.qt.io/download, https://www.qt.io/download-open-source

Download the Qt Online Installer for your operating system (macOS, Windows, Linux) 
(e.g., for Windows: qt-unified-windows-x64-4.5.2-online.exe).

Run the installer. I installed Qt in C:\Qt.

#### gRPC

Starting points: https://github.com/grpc/grpc, https://github.com/grpc/grpc/tree/master/src/cpp

Building gRPC using CMake with FetchContent, see https://github.com/grpc/grpc/tree/master/src/cpp#fetchcontent, did not work on my computer.

Rather, I used the instructions under https://github.com/grpc/grpc/tree/master/src/cpp#install-using-vcpkg-package

	git clone https://github.com/Microsoft/vcpkg.git
	cd vcpkg

I cloned vcpkg.git into C:\local, giving C:\local\vcpkg  

Note: this of course requires the installation of Git if this is not yet the case, see https://gitforwindows.org/

	./bootstrap-vcpkg.bat
	
	./vcpkg install grpc
	
The execution of this last command may take 0.5 hour to 1.5 hours or even more, depending on your computer.

Finally, I added the following folders to my PATH:

	C:\local\vcpkg\installed\x64-windows\tools\protobuf
	C:\local\vcpkg\installed\x64-windows\tools\grpc

#### ROS 2

I did not manage to install ROS 2 on Windows 11. I tried to follow the instructions at https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html. According to https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html#system-requirements, only Windows 10 is supported.

See below for an installation on Ubuntu 22.04 LTS.

#### corolib

Copy or clone corolib.git to your computer.

I originally developed corolib with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.
Now I am using Visual Studio 2022 and Qt Creator 10.0.1 on a Windows 11 laptop.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:

	File -> Open -> CMake...
	
and select the CMakeLists.txt in the top-level folder.

In the CMakeLists.txt in the top-level folder, adapt the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR C:/local/boost/boost_1_82_0)
	set(Boost_LIBRARY_DIR C:/local/boost/boost_1_82_0/stage/lib)

The Qt examples are not built from the top-level CMakeLists.txt file,
but have to be built from the .pro files in the subdirectories of examples/qt5.

Go to the examples/ subdirectories and start the executables.
(See README.md files in the corresponding source directories.)

### On Ubuntu 22.04 LTS

I have tested corolib on an Ubuntu MATE 22.04 installation in VirtualBox. 
The g++ version was 11.2.0.

Currently I am using Ubuntu 22.04 LTS in WSL on a Windows 11 laptop.
The g++ version is 11.3.0.

The g++ version should be >= 10 to have coroutine support.)

#### CMake

Install cmake (if not yet done):

	sudo apt install -y cmake

#### Boost

Install Boost:

	sudo apt-get update
	sudo apt-get -y install libboost-dev

The Boost include files are in

	/usr/include/boost
	
On my computer, the Boost libraries (version 1.74.0) are located in

	/usr/lib/x86_64-linux-gnu/
	
In the CMakeLists.txt in the top-level folder, adapt (if necessary) the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR /usr/include/boost)
	set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)

#### Qt

To install and configure qtcreator on Ubuntu 22.04, follow the instructions described in
https://askubuntu.com/questions/1404263/how-do-you-install-qt-on-ubuntu22-04:

	sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake
	
#### gRPC

Starting points: https://github.com/grpc/grpc, https://github.com/grpc/grpc/tree/master/src/cpp

Building gRPC using CMake with FetchContent, see https://github.com/grpc/grpc/tree/master/src/cpp#fetchcontent, did not work on my computer.

Rather, I used the instructions under https://grpc.io/docs/languages/cpp/quickstart/#install-grpc

	export MY_INSTALL_DIR=$HOME/.local
	mkdir -p $MY_INSTALL_DIR
	export PATH="$MY_INSTALL_DIR/bin:$PATH"
	
	sudo apt install -y build-essential autoconf libtool pkg-config
	
Clone the grpc repo and its submodules to a local directory:

	git clone --recurse-submodules -b v1.55.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc
	
	cd grpc
	mkdir -p cmake/build
	pushd cmake/build
	cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
      ../..
	make -j 4
	make install
	popd


#### ROS 2

Please follow the instructions at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

#### corolib

Copy or clone corolib.git to your computer.

I propose to build corolib as follows:

	cd <path>/corolib
	mkdir build
	cd build
	cmake ../
	make -j 4

The Qt examples are not built from the top-level CMakeLists.txt file,
but have to be built from the .pro files in the subdirectories of examples/qt5.

Go to the examples/ subdirectories and start the executables. 
(See README.md files in the corresponding source directories.)

## Organization of corolib

The GitHub repository contains the library itself and several examples.

The README.md file in include/corolib gives more information on each of the classes of corolib.
Folder lib contains the source (.cpp) files for the header files (.h) in include/corolib.

The examples folder contains a README.md file with more information on the examples.

## Why coroutines?

Reason 1: Coroutines allow writing (distributed) applications using a natural synchronous style,
but with the efficiency benefits of asynchronous execution.

An application running on one processor can invoke an operation in an application on the same or another processor 
in a synchronous or asynchronous way.

* Using a synchronous operation, the application will block until the remote application has replied.

    In the meantime, the application cannot respond to new requests unless new requests are processed on a separate thread or the operation 
itself runs on a separate thread.

* Using an asynchronous operation, the application can start the operation by sending a request to the remote application, 
proceed with other tasks that do not depend on the response and then await the response 
either using polling or using a callback function that is called from a local or global event loop.

    The reader is referred to reading/CORBA-AMI.md for more information on this approach.
	
* Using an asynchronous operation in combination with coroutines, the callback function can be hidden in the library (corolib in this case) 
and the response will be returned by the co_await statement.
Usually this co_await statement will be placed in the coroutine in which the asynchronous operation was invoked.

   The callback function will pass the result of the remote operation invocation to the application and resume it.
The application will then continue its flow and invoke new asynchronous operations.
This is a more natural place than handling the response in the callback function from which new operations will be invoked.

   The reader is also referred to examples/why-coroutines for examples demonstrating the three approaches in several stand-alone applications.
   
Reason 2: Coroutines can be used as an alternative for threads.

Two or more coroutines can be started one after the other.
Each of these coroutines can implement a potentially infinite loop, invoking asynchronous requests
and then processing their response in the same code block inside the loop.
These coroutines may thus never co_return.
When suspended for the first time,
they pass control to the coroutine or function that started them and that will eventually pass control to an event loop.
Yet everything runs on the same thread, with coroutines proceding in an interleaved and colloraborative way.

## Coding style

I will use the following coding style in almost all example applications:

```c++
	// Start an asynchronous operation on a (remote) object.
	async_operation<aType> retObj = proxy_to_object.start_operation(in1, in2);
	// Do some other things that do not rely on the result of the operation.
	// Finally co_await the result if nothing else can be done in this coroutine.
	aType returnVal = co_await retObj; 
```

instead of using only a single line:

```c++
	aType returnVal = co_await proxy_to_object.start_operation(in1, in2);
```

There are several reasons:

1. Because we are dealing with distributed systems,
the first statement starts an operation on a remote object.
While the remote object is processing the request and preparing the reply,
the application can proceed with actions that do not rely on the result.

    This style can be further exploited by placing the first statement a bit higher in the code, 
above all statements that do not depend on the result, but at a place where all inputs are available.
Several operations can be started one after the other and the results can then be processed afterwards.

2. When learning coroutines, the first approach shows more clearly what is going on.
It shows the return type of the asynchronous operation, making it easier to find and examine the implementation.

3. An explicit declaration makes it easier to await the result of several operations using wait_all or wait_any.

4. corolib allows reusing the return object (retObj in this case)
for several asynchronous operation invocations that return an object of the same type.
Therefore the object has to be declared explicitly.

5. corolib allows declaring an async_operation<aType> object up-front and starting the asynchronous operation afterwards.
In between the declaration and the start (or after the start), one or more coroutines can co_wait on the async_operation<aType> object.
All coroutines that co_await-ed the async_operation<aType> object will be resumed when the asynchronous operation completes.

    The reader is referred to examples\various-boost\timer03.cpp and examples\various-qt\timer03.cpp for examples.

6. The proposed style is closer to the polling style used in CORBA AMI.

    The reader is referred again to reading/CORBA-AMI.md for more information on this approach.

## Comparison with other libraries

corolib is only a small library, but it allows to write a lot of client-server applications.
For a more comprehensive library, see https://github.com/lewissbaker/cppcoro
Some code in corolib has been inspired by cppcoro.

The approach in corolib is different from cppcoro in the following ways:
* corolib is meant as a library to learn about coroutines in C++, not as a means for evaluating the coroutines TS.
That is why there are many print statements in the code: running the programs with priority level 2 reveals the flow of control when using coroutines.
The output also shows that all coroutines run on the same thread but that several coroutines run in an interleaved way.
* I wanted to make a strict separation between the coroutine part and the asynchronous communication library.
This explains the use of Boost ASIO and Qt: the code in these libraries is not "aware" that is used by coroutines.

## Presentations

* Belgian C++ Users Group, 29 January 2020: Slides + code samples: http://becpp.org/blog/wp-content/uploads/2020/02/Johan-Vanslembrouck-Coroutines-in-C20.zip
An updated version of the code samples can be found in examples/corolab.
* Meeting C++ Online, 7 September 2022: Slides: https://meetingcpp.com/mcpp/slides/2022/Corolib-DistributedProgrammingWithC++Coroutines1807.pdf

## Contact

Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
