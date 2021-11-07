# corolib
 A C++ coroutine library for asynchronous distributed applications

The 'corolib' library demonstrates how C++ coroutines can be used to write asynchronous distributed applications.

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

The library (include/corolib) has 2 parts:
* Files async_operation.h, async_task.h, auto_reset_event.h, commservice.h, oneway_task.h, print.h, semaphore.h, wait_all_awaitable.h, wait_all_counter.h, wait_any.h and wait_any_awaitable.h are independent of Boost and are also used by the Qt examples.
* Files commclient.h, commcore.h and commserver.h use boost/asio and are only used by the Boost examples.
Folder lib contains the source (.cpp) files for the header files (.h) in include/corolib.

The examples have also 2 parts:
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
