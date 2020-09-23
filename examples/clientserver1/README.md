# Three-tier client-server application

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs1-server' is a pure server application with 'cs1-clientserver' as its client.
'cs1-clientserver' is a combined client/server application: it a a client for 'cs1-server' and it acts as a server for any of the 4 pure client applications: 'cs1-client1', 'cs1-client3', 'cs1-client3WA' and 'cs1-client3WAny'.

After building, launch the applications in the following order:

* 'cs1-server' and 'cs1-clientserver' (in any order)
* 'cs1-client1' (or one of its variants)(one or more instances) and/or 'cs1-client3' (one or more instances) and/or 'cs1-client3WA' (one or more instances) and/or 'cs1-client3WAny' (one or more instances)

For every incoming client request, 'cs1-clientserver' makes an interaction with 'cs1-server' that runs 
independently from the interaction with the client.

client1a.cpp is a variant of client1.cpp, using an auxiliary main_loop() function the async_task::get() function to retrieve the result.
Because of the possible use of a semaphore in this single-threaded application, get() can only be safely called after a co_await on the async_task object.
This explains the following lines:

		int ret1 = co_await task;
		// some other code
		int ret2 = task.get();

client1b.cpp is a variant of client1a.cpp (but without the use of async_task::get()) that does not use explicit async_operation objects.
For example, instead of writing

	async_operation<std::string> sr = start_reading();
	// some other code
	std::string strout = co_await sr;

client1b.cpp uses instead

	std::string strout = co_await start_reading();

I normally use the first style in all other code examples, for the following reasons:
* It shows the return type of the operation (can be used for "educational" purposes).
* It allows the coroutine to perform some tasks after the asynchronous operation has been started but before its result is needed and therefore has to be co_awaited upon.
* It allows to start several asynchronous operations one after the other and co_await their results afterwards, either one by one or using a wait_all_awaitable or a wait_any_awaitable object.

If none of these reasons apply, the second style saves some typing and leads to more concise code.

client1c.cpp declares async_operation objects that go out of scope, for example

	{
		// Reading
		async_operation<std::string> sr = start_reading();
	}

When the operation completes, its callback function does not have any object anymore to complete/resume.
client1c.cpp shows that this does not lead to any problems.
