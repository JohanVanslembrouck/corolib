# corolib
 A C++ coroutine library for asynchronous distributed applications

The 'corolib' library demonstrates how C++ coroutines can be used to write asynchronous distributed applications.

## Why coroutines?

Coroutines are a natural way to write efficient (distributed) applications in a natural way.

An application running on one processor can start an operation in an application on the same or another processor 
in a synchronous or asynchronous way.

* Using a synchronous operation, the application will block until the operation has responded.

    In the meantime, the application cannot respond to new requests unless new requests are processed on a separate thread or the operation itself runs on a separate thread.

* Using an asynchronous operation, the application can start the operation by sending a request to the remote application, 
proceed with other tasks that do not depend on the response, and then await the response 
in an event loop where other inputs can be handled as well.

    The response will usually be handled by a piece of code that is typically located in a callback function registered at the invocation of the operation. A disadvantage is that starting the operation (sending the request) and processing the response are at different places (functions) in the code. Local variables in the function that started the operations are not accessesible in the callback function but they can be passed in a lambda closure. After having processed the response in the callback function, and depending on the response, the callback function will typically start another operation and pass another callback function to process the response. This leads to a chain of callback functions, with the first part processing the response of one operation  and the second part continuing the program flow. The callback functions are not easily reusable to implement other applications.

* Using an asynchronous operation in combination with coroutines, a generic callback function can be hidden in the library (corolib in this case) 
and the response can be handled after the co_await statement in the coroutine in which the asynchronous operation was originally invoked.

   The callback function will pass the result of the operation to the application flow and resume it. Depending on the result, the main application will then continue its flow and invoke a new asynchronous operation. This is a more natural place than handling the response in the callback function.

In short: the use of coroutines allow asynchronous applications to be written using a synchronous (sequential) style.

Coroutines can also be used as a replacement for threads.

Two or more coroutines can be started one after the other.
Each of these coroutines can implement a potentially infinite loop, invoking asynchronous requests
and then processing their response in the same code block inside the loop.
These coroutines may thus never co_return, but when suspended for the first time,
they pass control to the coroutine that started them and that will eventually pass control to the event loop.
Yet everything runs on the same thread, with coroutines proceding in an interleaved and colloraborative way.

## Coding style

I will use the following coding style in almost all example applications:

	// Start an asynchronous operation on a (remote) object.
	async_operation<aType> retObj = proxy_to_object.start_operation(in1, in2);
	// Do some other things that do not rely on the result of the operation.
	// Finally co_await the result if nothing else can be done.
    aType returnVal = co_await retObj; 

instead of using only a single line:

	aType returnVal = co_await proxy_to_object.start_operation(in1, in2);

There are several reasons:

1. Because we are dealing with distributed systems,
the first statement starts an operation on a remote object.
While the remote object is processing the request and preparing the reply,
the application can proceed with actions that do not rely on the result.

    This style can be further exploited by placing the first statement a bit higher in the code, above all statements that do not depend on the result, but at a place where all inputs are available.
Several operations can be started one after the other and the results can then be processed afterwards.

2. When learning coroutines, the first approach more clearly shows what is going on.
It shows the return type of the asynchronous operation, making it easier to find and examine the implementation.

3. An explicit declaration makes it easier to await the result of several operations using wait_all or wait_any.

4. corolib allows reusing the return object (retObj in this case)
for several asynchronous operation invocations that return an object of the same type.
Therefore the object has to be declared explicitly.

## Communication libraries and frameworks

For its communication 'corolib' uses the Boost ASIO library or Qt (QTcpSocket, QTcpServer).
I used Boost 1.70 and Qt 5.14.2, but many other versions will do.

I developed 'corolib' with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:

	File -> Open -> CMake...
	
and select the CMakeLists.txt in the top-level folder.

In the CMakeLists.txt in the top-level folder, adapt the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR C:/Boost/include/boost-1_70)
	set(Boost_LIBRARY_DIR C:/Boost/lib)
	
The Qt examples are not built from the top-level CMakeLists.txt file,
but have to be built from the .pro files in examples/clientserver11.

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
* Folder various-boost contains stand-alone applications that use Boost. It contains only a timer application at this moment.
* Folder various-qt contains stand-alone applications that use Qt. It contains only a timer application at this moment.

## Comparison with other libraries

'corolib' is only a small library, but it allows to write a lot of client-server applications.
For a more comprehensive library, see https://github.com/lewissbaker/cppcoro
Some code in 'corolib' has been inspired by 'cppcoro'.

The approach in 'corolib' is different from 'cppcoro' in the following ways:
* 'corolib' is meant as a library to learn about coroutines in C++, not as a means for evaluating the coroutines TS.
That is why there are many print statements in the code: running the programs with priority level 2 reveals the flow of control when using coroutines.
The output also shows that all coroutines run on the same thread but that several coroutines run in an interleaved way.
* I wanted to make a strict separation between the coroutine part and the asynchronous communication library.
This explains the use of Boost ASIO and Qt: the code in these libraries is not "aware" that is used by coroutines.

Contact: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
