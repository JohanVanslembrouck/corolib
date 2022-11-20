# corolib

A C++ coroutine library for writing asynchronous distributed applications.

## Installation and building

Copy or clone corolib.git to your computer.

### Prerequisites

For its communication corolib uses the Boost ASIO library or Qt (QTcpSocket, QTcpServer).
I originally used Boost 1.70 (now Boost 1.79) and Qt 5.14.2, but many other versions will do.

### Building on Windows 10

I developed corolib with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:

	File -> Open -> CMake...
	
and select the CMakeLists.txt in the top-level folder.

In the CMakeLists.txt in the top-level folder, adapt the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR C:/local/boost/boost_1_79_0)
	set(Boost_LIBRARY_DIR C:/local/boost/boost_1_79_0/stage/lib)

The Qt examples are not built from the top-level CMakeLists.txt file,
but have to be built from the .pro files in examples/clientserver11.

### Building on Linux

I have tested corolib on an Ubuntu MATE 22.04 installation. The g++ version is 11.2.0.
(The g++ version should be >= 10 to have coroutine support.)

You have to install cmake (if not yet done):

	sudo apt install cmake

and boost:

	sudo apt-get update
	sudo apt-get -y install libboost-dev

The Boost include files are in

	/usr/include/boost
	
On my machine, the Boost libraries (version 1.74.0) are located in

	/usr/lib/x86_64-linux-gnu/

I propose to build corolib as follows:

	cd <path>/corolib
	mkdir build
	cd build
	cmake ../
	make

Go to the examples/ subdirectories and start the executables. 
(See README.md files in the corresponding source directories.)

To install and configure qtcreator on Ubuntu 22.04, follow the instructions at
https://askubuntu.com/questions/1404263/how-do-you-install-qt-on-ubuntu22-04
	

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


## Organization of corolib

The GitHub repository contains the library itself and several examples.

The library (include/corolib) has two parts:
* Files async_operation.h, async_task.h, auto_reset_event.h, commservice.h, oneway_task.h, print.h, 
semaphore.h, wait_all_awaitable.h, wait_all_counter.h, wait_any_one.h and wait_any_awaitable.h 
are independent of Boost and are also used by the Qt examples. 
The README file in include/corolib gives more information on each of the classes.
* Files commclient.h, commcore.h and commserver.h use boost/asio and are only used by the Boost examples.

Folder lib contains the source (.cpp) files for the header files (.h) in include/corolib.

The examples folder has the following subfolders:
* Folder tutorial contains various standalone examples showing the use of corolib but without use of Boost or Qt.
* Folders clientserver1, clientserver2, clientserver3 and clientserver4 use Boost for client-server communication.
Two or more applications have to be started, e.g. a server application and one or more client applications.
* Folder clientserver11 uses Qt and folder common-qt defines classes that are used by clientserver11 
(and other examples that may be added in the future).
* Folder various-boost contains stand-alone applications that use Boost. At this moment it contains only a timer application.
* Folder various-qt contains stand-alone applications that use Qt. Again, at this moment it contains only a timer application.
* Folder why-coroutines contains various examples that explain the advantages of C++ coroutines for writing (distributed) applications.
* Folder corolab contains examples that do not use the corolib include files or libraries, but were the basis from where corolib has been "distilled."

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

Contact: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
