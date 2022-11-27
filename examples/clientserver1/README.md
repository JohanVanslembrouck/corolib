# Three-tier client-server application

server.cpp (executable: cs1-server.exe or cs1-server) is a pure server application with clientserver.cpp as its client.

clientserver.cpp (cs1-clientserver) is a combined client/server application: 
it is a client for server.cpp and it acts as a server for any of the pure client applications, see below.

For every incoming client request, cs1-clientserver makes an interaction with cs1-server that runs 
independently from the interaction with the client(s).

client1.cpp (cs1-client1) is the base client application. 
The mainflow() coroutine connects to the server, writes a string to the server, reads the response and closes the connection.
This sequence is repeated 100 times.

client1a.cpp (cs1-client1a) is a variant of client1.cpp, using an auxiliary main_loop(int i, int& counter) function 
and async_task::get_result() in main_loop() to retrieve the result.

Because of the possible use of a semaphore in this single-threaded application, 
get_result() can only be safely called after a co_await on the async_task object.

This explains the following lines:

```c++
		int ret1 = co_await task;
		// some other code
		int ret2 = task.get_result();
```

client1b.cpp (cs-client1b) is a variant of client1a.cpp (but without the use of async_task::get_result()) 
that does not use explicit async_operation objects.

For example, instead of writing

```c++
	async_operation<std::string> sr = start_reading();
	// some other code
	std::string strout = co_await sr;
```

client1b.cpp uses instead

```c++
	std::string strout = co_await start_reading();
```

I normally use the first style in all other code examples, for the following reasons:

* It shows the return type of the operation (can be used for "educational" purposes).
* It allows the coroutine to perform some tasks after the asynchronous operation has been started but before its result is needed and therefore has to be co_awaited upon.
* It allows to start several asynchronous operations one after the other and co_await their results afterwards, either one by one or using a when_all or a when_any object.

If none of these reasons apply, the second style saves some typing and leads to more concise code.

client1c.cpp (cs1-client1c) declares async_operation objects that go out of scope, for example

```c++
	{
		// Reading
		async_operation<std::string> sr = start_reading();
	}
```

When the operation completes, its callback function does not have any object anymore to complete/resume.
client1c.cpp shows that this does not lead to any problems.

client3.cpp (cs1-client3) uses 3 CommClient objects.
The client application starts asynchronous operations on the 3 CommClient objects one after the other and then
co_awaits their completion, again one after the other.

client3WA.cpp (cs1-client3WA) is a variant of client3.cpp that uses when_all to co_await the completion of the 3 async_operations using a single line.

client3WAny.cpp (cs1-client3WAny) is a variant of client3WA.cpp that uses when_any to co_await the completion of the 3 async_operations using a single line.
Because 3 async_operations have to complete, a loop is used to co_await the completion of all 3 asynchronous operations.

client4obs.cpp (cs1-client4obs) uses 4 observer coroutines. This example relies on the definition

```c++
#define RESUME_MULTIPLE_COROUTINES 1
```
in async_operation.h. Before starting the asynchronous read operation, mainflow() starts 4 observer coroutines that each process the read response
from the server. mainflow() co_awaits the completion of the observer coroutines using when_all.

After building, launch the applications in the following order:

* 'cs1-server' and 'cs1-clientserver' (in any order)
* 'cs1-client1' (or one of its variants cs1-client1a, cs1-client1b or cs1-client1c)(one or more instances) and/or 
* 'cs1-client3' (one or more instances) and/or 
* 'cs1-client3WA' (one or more instances) and/or
* 'cs1-client3WAny' (one or more instances) and/or
* 'cs1-client4obs' (one or more instances)
