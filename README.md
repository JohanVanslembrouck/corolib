# corolib

corolib is a C++ coroutine library for writing asynchronous distributed applications.

## Objectives

corolib has two main objectives:

* Demonstrate that it is possible to write asynchronous distributed applications in a synchronous style using various (open source) asynchronous communication frameworks.

  * Asynchronous operations can be completed on the same thread they are started on (completion functions are called from an event loop) or on another thread.
  * There is no need to make changes to these frameworks or to the corolib library, but sometimes an "adaptation layer" is useful. This layer contains functionality that would otherwise be replicated in the applications.
  
* Learning about and experimenting with coroutines

  * Directory examples/corolab (short for "coroutine laboratory") contains simple stand-alone programs that also include a coroutine definition; the reader just has to open a single file to have access to all source code.
  * Directory examples/tutorial contains many examples using one or more corolib classes (async_task, async_operation, ...), but without using any communication framework.
  * Directory examples/why-coroutines explains why you should consider the use of coroutines, by comparing three programming styles: synchronous, asynchronous (with callbacks) and coroutines.
  * Directory examples/transform explains how coroutines are transformed or lowered to non-coroutine C++ code.
  * corolib contains a dedicated print function, taking a print level as first argument; this allows following the control flow of a coroutine program at various levels: application level, corolib level, user-specific level, ...
  * corolib contains tracking functions that allow verifying whether all coroutine return type objects, operation objects, promise objects and final awaiter objects are released.
  * Class async_task can be tailored, allowing to experiment with the three await_suspend return types (void, bool, std::coroutine_typs<>), so that the user can compare the pros and cons of these variants.

## Installation and building

For its communication, corolib currently uses any of the following frameworks:

* Boost ASIO library
* Qt (QTcpSocket, QTcpServer)
* gRPC
* ROS2 (Humble)
* TAO (The ACE ORB)
* cppcoro (WIN32 API) (included in corolib)

You do not have to install Boost, Qt, gRPC, ROS2 or TAO on your computer before building corolib from the top-level CMakeLists.txt file.
Without any of these libraries, you can still build and run many examples illustrating the use of corolib.

You can install any of the mentioned communication libraries one by one and then include them in the build by setting a flag in the top-level CMakeLists.txt file.
The installation of these libraries is described below.

### On Windows 10 or 11

#### corolib

Copy or clone corolib.git to your computer.

I originally developed corolib with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.
Now I am using Visual Studio 2022 and Qt Creator 10.0.1 on a Windows 11 laptop.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:

	File -> Open -> CMake...
	
and select the CMakeLists.txt in the top-level folder.

Go to the examples/ subdirectories and start the executables.
(See README.md files in the corresponding source directories.)

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

In the top-level CMakeLists.txt, adapt BOOST_ROOT to your own installation of the Boost library:

    set(BOOST_ROOT C:/local/boost/boost_1_82_0/)

After installing Boost ASIO, set the BOOST_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

#### Qt

Starting points: https://www.qt.io/download, https://www.qt.io/download-open-source

Download the Qt Online Installer for your operating system (macOS, Windows, Linux) 
(e.g., for Windows: qt-unified-windows-x64-4.5.2-online.exe).

Run the installer. I installed Qt in C:\Qt.

After installing Qt5, set the QT5_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

You can build the Qt examples from the top-level CMakeLists.txt file.
In addition, you can also build them from the qt5.pro file in the top-level directory.

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

After installing gRPC, set the GRPC_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

#### ROS 2

I did not manage to install ROS 2 on Windows 11. I tried to follow the instructions at https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html. According to https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html#system-requirements, only Windows 10 is supported.

See below for an installation on Ubuntu 22.04 LTS.

#### TAO

Starting point: https://download.dre.vanderbilt.edu/

Download the file ACE+TAO-7.1.1.zip and place it in a directory (e.g.) C:\local. 
This is the directory that already contains boost and vcpkg, see above.

Unzip the file. This will produce a directory ACE_wrappers.
Consequently, the full path is C:\local\ACE_wrappers.
I will refer to this directory as ...\ACE_wrappers from now on.

Open the file ...\ACE_wrappers\ACE-INSTALL.html in a web browser and navigate to the section 
"Building and Installing ACE on Windows" and, in case of the use of Visual Studio, to 
"Building and Installing ACE on Windows with Microsoft Visual Studio".

Follow the instructions in that section. In short:

In directory ...\ACE_wrappers\ace, create a file called config.h and enter the following line in this file:

    #include "ace/config-win32.h"

Open the file ..\ACE_wrappers\ACE_vs2019.sln with Visual Studio 2019 or 2022 (or ACE_vs2017.sln with Visual Studio 2017).

Visual Studio 2022 reports the use of an earlier version of the Visual Studio platform toolset 
and proposes to upgrade the projects to target the latest Microsoft toolset. I upgraded to the latest toolset.

Build the solution. This builds ACE (The ADAPTIVE Communication Environment). We still have to build TAO (The ACE ORB).

Open the file ...\ACE_wrappers\TAO\TAO-INSTALL.html in a web browser and follow the instructions in section 
"On Windows NT and Windows 2000 and Windows XP". In short:

Open the file ..\ACE_wrappers\TAO\TAO_vs2019.sln with Visual Studio and build the solution.

Edit the Environment Variables and add the following directories to the Path variable:

	C:\local\ACE_wrappers\lib
	C:\local\ACE_wrappers\bin
	C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\<VERSION>\bin\Hostx64\x64

The VERSION changes with every upgrade of Visual Studio (2022).

The last line is needed if you want to run the TAO IDL compiler (tao_idl in C:\local\ACE_wrappers\bin) from the command line.
If the last line is not present, running tao_idl will fail with the message that CL.exe is not found.

After installing TAO, set the TAO_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

### On Ubuntu 22.04 LTS

I have tested corolib on an Ubuntu MATE 22.04 installation in VirtualBox. 
The g++ version was 11.2.0.

Currently I am using Ubuntu 22.04 LTS in WSL on a Windows 11 laptop.
The g++ version is 11.3.0.

The g++ version should be >= 10 to have coroutine support.

#### CMake

Install cmake (if not yet done):

	sudo apt install -y cmake

#### corolib

Copy or clone corolib.git to your computer.

I propose to build corolib as follows:

	cd <path>/corolib
	mkdir build
	cd build
	cmake ../
	make -j 4

Go to the examples/ subdirectories and start the executables. 
(See README.md files in the corresponding source directories.)

#### Boost

Install Boost:

	sudo apt-get update
	sudo apt-get -y install libboost-dev

The Boost include files are in

	/usr/include/boost
	
On my computer, the Boost libraries (version 1.74.0) are located in

	/usr/lib/x86_64-linux-gnu/

After installing Boost ASIO, set the BOOST_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

#### Qt

To install and configure qtcreator on Ubuntu 22.04, follow the instructions described in
https://askubuntu.com/questions/1404263/how-do-you-install-qt-on-ubuntu22-04:

	sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake

After installing Qt5, set the QT5_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

You can build the Qt examples from the top-level CMakeLists.txt file.
In addition, you can also build them from the qt5.pro file in the top-level directory.

#### gRPC

Starting points: https://github.com/grpc/grpc, https://github.com/grpc/grpc/tree/master/src/cpp

Building gRPC using CMake with FetchContent, see https://github.com/grpc/grpc/tree/master/src/cpp#fetchcontent, did not work on my computer.

Rather, I used the instructions under https://grpc.io/docs/languages/cpp/quickstart/#install-grpc

	export MY_INSTALL_DIR=$HOME/.local
	mkdir -p $MY_INSTALL_DIR
	export PATH="$MY_INSTALL_DIR/bin:$PATH"
	
	sudo apt install -y build-essential autoconf libtool pkg-config

Note: I used MY_INSTALL_DIR=$HOME/grpcbin as installation directory.

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

Note: I cloned grpc to $HOME/grpcsrc.

Add the following lines to .bashrc for future use:

    export PATH=$HOME/.local/bin:$PATH

or, in my setup:

    export PATH=$HOME/grpcbin/bin:$PATH

After installing gRPC, set the GRPC_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

#### ROS 2

Please follow the instructions at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

#### TAO

Starting point: https://download.dre.vanderbilt.edu/

Download the file ACE+TAO-7.1.1.tar.gz and place it in (e.g.) your home directory, $HOME.
Unzip the file:

    tar xvf ACE+TAO-7.1.1.tar.gz

This will produce a directory ACE_wrappers.
Consequently, the full path is $HOME\ACE_wrappers.

Open the file $HOME\ACE_wrappers\ACE-INSTALL.html in a web browser and navigate to the section 
"Building and Installing ACE on UNIX".

Follow the instructions in that section. In short:

Define the ACE_ROOT environment variable:

    export ACE_ROOT=$HOME/ACE_wrappers 

Create a configuration file, $ACE_ROOT/ace/config.h, 
that includes the appropriate platform/compiler-specific header configurations from the ACE source directory:

    #include "ace/config-linux.h"

Create a build configuration file, $ACE_ROOT/include/makeinclude/platform_macros.GNU,
that contains the appropriate platform/compiler-specific Makefile configurations, e.g.,

    include $(ACE_ROOT)/include/makeinclude/platform_linux.GNU

Set LD_LIBRARY_PATH to the directory where the binary version of the ACE library will be built into:

    export LD_LIBRARY_PATH=$ACE_ROOT/lib:$LD_LIBRARY_PATH

In a terminal window, navigate to $ACE_ROOT and build ACE:

    cd $ACE_ROOT
    make

Building ACE in my WSL environment took about 10 minutes.

Open the file ...\ACE_wrappers\TAO\TAO-INSTALL.html in a web browser and follow the instructions in section 
"On UNIX platforms". In short:

Build GPERF as follows:

    cd $ACE_ROOT/apps/gperf/src
    make

Set the TAO_ROOT environment variable to $ACE_ROOT/TAO:

    export TAO_ROOT=$ACE_ROOT/TAO

Build TAO:

    cd $TAO_ROOT
    make

Building TAO takes a long time (about 2 hours in my WSL environment). I did not use e.g. make -j 4.

Add the following lines to .bashrc:

    export ACE_ROOT=$HOME/ACE_wrappers
    export TAO_ROOT=$ACE_ROOT/TAO
    export PATH=$ACE_ROOT/bin:$PATH
    export LD_LIBRARY_PATH=$ACE_ROOT/lib:$LD_LIBRARY_PATH

After installing TAO, set the TAO_INSTALLED flag to TRUE in the top-level CMakelists.txt file.

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

corolib is a rather small library, because it completely separates the coroutine part from the communication frameworks.
Some code in corolib has been inspired by  https://github.com/lewissbaker/cppcoro

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

Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
