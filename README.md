# corolib
 A C++ coroutine library for asynchronous distributed applications

The 'corolib' library demonstrates how C++ coroutines can be used to write asynchronous distributed applications.

Coroutines are a natural way to write efficient distributed applications.
An application running on one processor can start an operation on another processor or computer (remote method invocation).
* Using a synchronous operation, the application will block until the operation has responded.
In the meantime, the application cannot respond to new requests unless this request is processed in a separate thread 
or the operation itself runs on a separate thread.
* Using an asynchronous operation, the application can start the operation, 
proceed with other tasks that do not depend on the response, and then await the response 
in an event loop where other inputs can be handled as well.
The response will usually be handled by a piece of code that is typically located in a callback function 
registered at the invocation of the operation.
A disadvantage is that starting the operation (sending the request) and processing the response are at different places in the code.
* Using an asynchronous operation in combination with coroutines, the callback function can be hidden in the library (corolib in this case) and
the response can be handled after the co_await statement in the coroutine in which the asynchronous operation was originally invoked.
This is a more natural place than handling the response in a separate piece of code. The use of coroutines allow asynchronous applications
to be written using a synchronous (sequential) style.

Coroutines can also be used as a replacement for threads.
Two or more coroutines can be started one after the other.
Each of these coroutines can implement a potentially infinite loop, invoking asynchronous requests
and then processing their response in the same code block inside the loop.
These coroutines may thus never co_return, but when suspended for the first time,
they pass control to the coroutine that started them and that will pass control to the event loop.
Yet everything runs on the same thread, with coroutines proceding in an interleaved and colloraborative way.

For its communication 'corolib' uses the Boost ASIO library or Qt (QTcpSocket, QTcpServer). I used Boost 1.70 and Qt 5.14.2, but many other versions will do.

I developed 'corolib' with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:

	File -> Open -> CMake...
	
and select the CMakeLists.txt in the top-level folder.

In the CMakeLists.txt in the top-level folder, adapt the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR C:/Boost/include/boost-1_70)
	set(Boost_LIBRARY_DIR C:/Boost/lib)
	
The Qt examples are currently not built from the top-level CMakeLists.txt file, but have to be built from the .pro files in examples/clientserver11.

The GitHub repository contains the library itself and several examples.

The library (include/corolib) has two parts:
* Files async_operation.h, async_task.h, auto_reset_event.h, commservice.h, oneway_task.h, print.h, semaphore.h, wait_all_awaitable.h, wait_all_counter.h, wait_any.h and wait_any_awaitable.h are independent of Boost and are also used by the Qt examples. The README file in include/corolib gives more information for each of the classes.
* Files commclient.h, commcore.h and commserver.h use boost/asio and are only used by the Boost examples.
Folder lib contains the source (.cpp) files for the header files (.h) in include/corolib.

The examples have also two parts:
* Folders clientserver1, clientserver2, clientserver3 and clientserver4 use Boost.
* Folder clientserver11 uses Qt and folder common-qt defines classes that are used by clientserver11 (and other examples that may be added in the future).

'corolib' is only a small library (at the moment).
For a more comprehensive library, see https://github.com/lewissbaker/cppcoro
Some code in 'corolib' has been inspired by 'cppcoro', from which I learned a lot.

The approach in 'corolib' is different from 'cppcoro' in the following ways:
* 'corolib' is meant as a library to learn about coroutines in C++, not as a means for evaluating the coroutines TS.
That is why there are many print statements in the code: running the programs reveals the flow of control when using coroutines.
The output also shows that all coroutines run on the same thread but that several coroutines run in an interleaved way.
* I wanted to make a strict separation between the coroutine part and the asynchronous communication library.
This explains the use of Boost ASIO and Qt: the code in these libraries is not "aware" that is used by coroutines.

Contact: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
