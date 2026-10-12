# corolib

corolib is a C++ coroutine library for writing asynchronous distributed applications 
using Boost ASIO, Qt, gRPC, ROS2, TAO, ..., as communication frameworks.

## Objectives

corolib has two main objectives:

* Demonstrate that it is possible to write asynchronous distributed applications in a synchronous style using various (open source) asynchronous communication frameworks.

  * Asynchronous operations can be completed on the same thread they are started on (completion functions are called from an event loop) or on another thread.
  * There is no need to make changes to these frameworks or to the corolib library, but sometimes an "adaptation layer" is useful. This layer contains functionality that would otherwise be replicated in the applications.
  
* Learning about and experimenting with coroutines.

  * Directory [corolab](./studies/corolab) (short for "coroutine laboratory") contains simple stand-alone programs that also include a coroutine definition; the reader just has to open a single file to have access to all source code.
  * Directory [tutorial](./examples/tutorial) contains many examples using one or more corolib classes (async_task, async_operation, ...), but without using any communication framework.
  * Directory [why-coroutines](./studies/why-coroutines) explains why you should consider the use of coroutines for writing (distributed) asynchronous applications, by comparing three programming styles: synchronous, asynchronous (with callbacks) and coroutines.
  * Directory [why-coroutines2](./studies/why-coroutines2) contains examples that explain the advantages of C++ coroutines for writing asynchronous I/O-based applications.
  * Directory [transform](./studies/transform) explains how coroutines are transformed or lowered to non-coroutine C++ code.
  
 The reader is referred to [studies](./studies) for more examples and experiments.

 In addition:

  * corolib contains a dedicated print function, taking a print level as first argument; this allows following the control flow of a coroutine program at various levels: application level, corolib level, user-specific level, ...
  * corolib contains tracking functions that allow verifying whether all coroutine return type objects, operation objects, promise objects and final awaiter objects are released.
  * Class async_task can be tailored, allowing to experiment with the three await_suspend return types (void, bool, std::coroutine_typs<>), so that the user can compare the pros and cons of these variants.

## Installation and building

For its communication, corolib currently uses any of the following frameworks:

* Boost ASIO library
* Qt (QTcpSocket, QTcpServer)
* gRPC (Google Remote Procedure Call)
* cppcoro (source code included in corolib)
* libevent
* ROS2 (Humble) (ROS = Robot Operating System)
* TAO (The ACE (ADAPTIVE (A Dynamically Assembled Protocol Transformation, Integration and eValuation Environment)) ORB (Object Request Broker))
* curl

You do not have to install any of these frameworks/libraries on your computer 
before building corolib from the top-level CMakeLists.txt file.

Without any of these libraries, you can still build and run the tutorial examples illustrating the use of corolib.

You can install any of the mentioned communication libraries one by one
and then include them in the build by setting a flag in the top-level CMakeLists.txt file.

Please read [Installation and building on Windows](INSTALLATION_Windows.md) for instructions
on how to install and build corolib and its examples on Windows 10/11.

Please read [Installation and building on Ubuntu](INSTALLATION_Ubuntu.md) for instructions
on how to install and build corolib and its examples on Ubuntu 22.04/24.04/26.04.

After having built one or more communication frameworks,
run scripts/copy.bat (on Windows)
or scripts/copy.sh (on Ubuntu)
to copy configuration and run files to the folders with the executables.

## Organization of corolib

The GitHub repository contains the library itself and several examples and studies.

The README.md file in [corolib](./include/corolib) gives more information on each of the classes of corolib.
Folder [lib](./lib) contains the source (.cpp) files for the header files (.h) in [include](./include/corolib).

The [examples](./examples) folder contains a README.md file with more information on the examples.

The [studies](./studies) folder contains a README.md file with more information on the studies.

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

    The reader is referred to [CORBA-AMI](./docs/CORBA-AMI.md) for more information on this approach.
	
* Using an asynchronous operation in combination with coroutines, the callback function can be hidden in the library (corolib in this case) 
and the response will be returned by the co_await statement.
Usually this co_await statement will be placed in the coroutine in which the asynchronous operation was invoked.

   The callback function will pass the result of the remote operation invocation to the application and resume it.
The application will then continue its flow and invoke new asynchronous operations.
This is a more natural place than handling the response in the callback function from which new operations will be invoked.

   The reader is also referred to [why-coroutines](./studies/why-coroutines) for examples demonstrating the three approaches in several stand-alone applications.
   
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

2. When learning about coroutines, the first approach shows more clearly what is going on.
It shows the return type of the asynchronous operation, making it easier to find and examine the implementation.

3. An explicit declaration makes it easier to await the result of several operations using when_all or when_any.

4. corolib allows reusing the return object (retObj in this case)
for several asynchronous operation invocations that return an object of the same type.
Therefore the object has to be declared explicitly.

5. corolib allows declaring an async_operation<aType> object up-front and starting the asynchronous operation afterwards.
In between the declaration and the start (or after the start), one or more coroutines can co_wait on the async_operation<aType> object.
All coroutines that co_await-ed the async_operation<aType> object will be resumed when the asynchronous operation completes.

    The reader is referred to [boost-timer03](./examples/boost/various/timer03.cpp) and [qt5-timer03](./examples/qt5/various/timer03.cpp) for examples.

6. The proposed style is closer to the polling style used in CORBA AMI.

    The reader is referred again to [CORBA-AMI](./docs/CORBA-AMI.md) for more information on this approach.

## Comparison with other libraries

corolib is a rather small library, because it completely separates the coroutine part from the communication frameworks.
Some code in corolib has been inspired by  https://github.com/lewissbaker/cppcoro

The approach in corolib is different from cppcoro in the following ways:
* corolib is meant as a library to learn about coroutines in C++, not as a means for evaluating the coroutines TS.
That is why there are many print statements in the code: running the programs with print level 2 reveals the flow of control when using coroutines.
The output also shows that all coroutines run on the same thread but that several coroutines run in an interleaved way.
* I wanted to make a strict separation between the coroutine part and the asynchronous communication library.
This explains the use of Boost ASIO, Qt5, gRPC, etc.: the code in these libraries is not "aware" that is used by coroutines.

## Presentations

* Belgian C++ Users Group, 29 January 2020: "Coroutines in C++ 20" Slides + code samples: http://becpp.org/blog/wp-content/uploads/2020/02/Johan-Vanslembrouck-Coroutines-in-C20.zip
An updated version of the code samples can be found in examples/corolab.
* Meeting C++ Online, 7 September 2022: "Corolib: Distributed programming with C++ coroutines" Slides: https://meetingcpp.com/mcpp/slides/2022/Corolib-DistributedProgrammingWithC++Coroutines1807.pdf
* Belgian C++ Users Group, 15 January 2025: "Why use coroutines for asynchronous applications?" Slides: https://becpp.org/blog/wp-content/uploads/2025/01/Johan-Vanslembrouck-Why-Use-Coroutines-For-Asynchronous-Applications.pdf
* Meeting C++ 2025, 8 November 2025: "Why use coroutines for asynchronous applications?" Slides: https://meetingcpp.com/mcpp/slides/2025/WhyUseCoroutinesForAsynchronousApplications564642.pdf
* Belgian C++ Users Group, 2 December 2025: "Distributed programming with coroutines and asynchronous communication frameworks" Slides: https://becpp.org/blog/wp-content/uploads/2025/12/Johan-Vanslembrouck-DistributedProgrammingWithCoroutinesAndACFs.pdf

## License

Please read [License](LICENSE.md).

## Contact

Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
