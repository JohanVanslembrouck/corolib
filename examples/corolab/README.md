# corolab (coroutine laboratory)

Contains the examples I prepared for my presentation at the Belgian C++ Users Group on 29 January 2020.

See http://becpp.org/blog/wp-content/uploads/2020/02/Johan-Vanslembrouck-Coroutines-in-C20.zip and
http://becpp.org/blog/2019/12/17/next-becpp-ug-meeting-planned-for-january-29th-2020/

Almost all files are self-contained: all coroutine functionality is implemented in the file itself.

These examples has not been changed since then, except for the removal of "experimental".
They served as the basis for developing corolib, hence the name corolab or "coroutine laboratory".


## Examples using C++11

* p0010.cpp: C++11, future + promise + thread

* p0020.cpp: C++11, future + async

* p0030.cpp: C++11, condition variable + mutex + thread

* p0040.cpp: C++11, mutex + thread

* p0050.cpp: C++11, no condition variable, no mutex, thread

* p0060.cpp: C++11, future + async: eager and lazy start

* p0200.cpp: C++11, primes: with print in primes / with callback / with iterator

## Examples illustrating the use of co_return

* p0100.cpp: co_return: eager + lazy

* p0110.cpp: co_return: eager + lazy, one base class

* p0120.cpp: co_return: eager + lazy, one base class, chain of coroutine calls

## Examples illustrating the use of co_yield

* p0300.cpp: co_yield: generator

* p0310.cpp: co_yield: generator + iterator

* p0320.cpp: co_yield: recursive generator: uses recursive_generator from recursive_generator.cpp (cppcoro)

* p0330.cpp: co_yield + for co_await: asynchronous generator

    Notice that "for co_await" did not make it into the C++20 standard.
	
* p0340.cpp: co_yield + for co_await: asynchronous generator: stack overflow.
Uses the "compiler supports symmetric transfer" alternative from async_generator.hpp (cppcoro)

    Notice that "for co_await" did not make it into the C++20 standard.
	
* p0345.cpp: co_yield + for co_await: asynchronous generator.
Uses the "compiler does not support symmetric transfer" alternative from async_generator.hpp (cppcoro)

    Notice that "for co_await" did not make it into the C++20 standard.
	
## Examples illustrating the use of co_await

* p0400.cpp: co_await: suspend_always();

* p0403a.cpp: co_await, co_return: code generation: await_suspend() returns void

* p0403at.cpp: p0403a.cpp with traces

* p0403b.cpp: co_await, co_return: code generation: await_suspend() returns bool

* p0403bt.cpp: p0403b.cpp with traces

* p0403c.cpp: co_await, co_return: code generation: await_suspend() returns coroutine_handle

* p0403ct.cpp: p0403c.cpp with traces

* p0403d.cpp: await_suspend() returns void and resumes own coroutine

* p0403dt.cpp: p0403c.cpp with traces

* p0406.cpp: Coroutine type that does not return an object to its caller

* p0414.cpp:

* p0416.cpp:

* p0418.cpp:

* p0420.cpp: co_await, co_return, eager, chain of coroutine calls, uses promise type directly (not recommended), resumed from thread

* p0422.cpp: co_await, co_return, eager, chain of coroutine calls, uses dedicated type, resumed from thread

* p0424.cpp: co_await, co_return, eager, chain of coroutine calls, uses dedicated type, resumed from main()

* p0426.cpp: co_await, co_return, eager, chain of coroutine calls, uses dedicated type, two coroutines resumed from main()

* p0430.cpp: co_await, co_return , lazy, chain of coroutine calls

* p0435.cpp:

* p0440.cpp: co_await, co_return, future, chain of coroutine calls

* p0450.cpp: co_await, co_return, p0450_future, chain of coroutine calls

* p0460.cpp: co_await, co_return, std::thread in await_suspend, single_consumer_event

* p0470.cpp: co_await, co_return, std::thread in await_suspend


## Producer - consumer examples

* p0500.cpp: C++11, Single producer – single consumer queue, condition variables, producer and consumer run on separate thread

* p0510.cpp: Single producer – single consumer queue , coroutines, producer and consumer run on separate thread

* p0520.cpp: Single producer – single consumer queue, coroutines, producer and consumer run on the same thread


## Boost examples without coroutines

* p0700s.cpp: C++11, Boost ASIO, server application, chain of callback functions

* p0700c.cpp: C++11, Boost ASIO, client application, chain of callback functions

* p0710c.cpp: C++11, Boost ASIO, client application, no callback functions chain


## Boost examples with coroutines

* p0800s.cpp: Boost ASIO, echo application

* p0800cs.cpp: Boost ASIO, combined client and server application

* p0800c.cpp: Boost ASIO, client connecting to server

* p0810c.cpp: Boost ASIO, 3 clients connecting to server, co_awaiting the 3 replies one-by-one

* p0820c.cpp: Boost ASIO, 3 clients connecting to server, and co_awaiting all replies (“wait all”)

* p0830s.cpp: Boost ASIO, server sending N replies to every connected client

* p0830c.cpp: Boost ASIO, 3 clients connecting to server, and co_awaiting M replies
