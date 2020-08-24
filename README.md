# corolib
 A C++ coroutine library for asynchronous distributed applications

The 'corolib' library demonstrates how C++ coroutines can be used to write asynchronous distributed applications.

For its communication, 'corolib' uses the Boost ASIO library. I used Boost 1.70, but many other versions will do.

I developed 'corolib' with Visual Studio 2019 on a Windows 10 laptop.
Building makes use of the CMake support of Visual Studio:
	File -> Open -> CMake...
and select the CMakeLists.txt in the main folder.

In the CMakeLists.txt in the top level folder, adapt the following variables to your own installation of the Boost library:

	set(Boost_INCLUDE_DIR C:/Boost/include/boost-1_70)
	set(Boost_LIBRARY_DIR C:/Boost/lib)

'corolib' is only a small library (at the moment).
For a more comprehensive library, see https://github.com/lewissbaker/cppcoro
Some code in 'corolib' has been inspired by 'cppcoro', from which I learned a lot.

The approach in 'corolib' is different from 'cppcoro' in the following ways:
* 'corolib' is meant as a library to learn about coroutines in C++, not as a means for evaluating the coroutines TS.
That is why there are many print statements in the code: running the programs reveals the flow of control when using coroutines.
The output also shows that all coroutines run on the same thread but that several coroutines run in an interleaved way.
* I wanted to make a strict separation between the coroutine part and the asynchronous communication library.
This explains the use of Boost ASIO: the code in this library is not "aware" that is used by coroutines.

Contact: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
