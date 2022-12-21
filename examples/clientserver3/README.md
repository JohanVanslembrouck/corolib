# Two tier client-server application with layer of abstraction

In general, an application may receive inputs from one or more sources and react in a specific way to each input type.
This can be implemented as a switch in C/C++ or a table that associates each input type with a function or a coroutine
that will be called when an input arrives.
The advantage of using coroutines instead of functions is that the application may accept and handle new inputs
while it is still handling one or more previous inputs.
A long-running function may block the application from handling new inputs at the moment these inputs arrive,
unless the function is run on a separate thread.
With coroutines, several inputs may be processed in an interleaved way on the same thread.

The application may be a server-type application that receives its inputs from one or more clients,
or it may be a client-type application that first connects to one or more servers that may then send inputs to its connected client(s).
Note that client and server should be seen in the context of a TCP/IP client that connects to a TCP/IP server.
Once the client has connected to a server, the requests and replies may flow in any direction.
In the examples we will consider the first type of application,
i.e. a server application that has to respond to N (4 in our case) input types from its clients.

In the applications the API does not use the received strings directly: 
a tiny abstraction layer has been added so that the application can use user-defined types that 
(in a real-life application) have to be marshalled onto or unmarshalled from \n-terminated strings.
Such real-life applications could use JSON data structures that, when marshalled to a string, do not contain \n inside.

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs3-server', 'cs3-server2', ... 'cs3-server9' are server applications with 'cs3-client1' or 'cs3-client2' as their client application.

After building, launch the applications in the following order:

* 'cs3-server' or 'cs3-server2' or ... or 'cs3-server9'
* 'cs3-client1' (one or more instances) or 'cs3-client2' (one or more instances)

cs3-server (server.cpp) and cs3-server2 (server2.cpp) are equivalent. 
The difference is that server.cpp is self-contained, while server2.cpp includes serverrequest.h and dispatcher.h.

cs3-server3 (server3.cpp) is an extension of cs3-server2 (server2.cpp).
The difference with server2.cpp is the use of a dedicated coroutine read_client_request that is co_awaited in mainflow_one_client.

cs3-server4 (server4.cpp) is an extension of cs3-server3 (server3.cpp).
It uses a different Dispatcher implementation than server3.cpp and server2.cpp. 
After having dispatched an operation (passed as a lambda), the dispatcher also completes an asynchronous operation. 
In this way main_one_client gets feedback on the invoked operation and can (if applicable) take some additional actions.

cs3-server5 (server5.cpp) is a variant of cs3-server4 (server4.cpp).
It allows coroutine mainflow_one_client to follow the progress of coroutine read_client_request by passing it a struct process_info_t.
In contrast to server4.cpp, server5.cpp does not use when_any.
Therefore, it does not have to save the return value of the dispatcher.registerFunctor calls.

cs3-server6 (server6.cpp) is based on cs3-server4 (server4.cpp).
A coroutine is passed as lambda (instead of a normal function in server4.cpp): it uses co_await and co_return in its implementation.
A different version of ServerRequest is used, where the operations return async_task<int> instead of oneway_task.
The passed lambda can therefore be co_awaited for in the dispatcher (dispatcher3.h).

cs3-server7 (server7.cpp) is a simplification of cs3-server6 (server6.cpp).
Because we can now co_await the lambda passed to the dispatcher,
the asynchronous operation that is completed by the dispatcher (in server4.cpp, server5.cpp and server6.cpp) has become obsolete.
This operation has been removed in server7.cpp.

cs3-server8 (server8.cpp) is based on cs3-server7 (server7.cpp).
server8.cpp uses "observer" couroutines, where one observer couroutine waits for and handles 1 specific request.
server8.cpp resumes the observer coroutine that is interested in that request.

cs3-server9 (server9.cpp) is based on cs3-server8 (server8.cpp).
In contrast to cs3-server8.cpp, cs3-server9.cpp uses a coroutine "chain" from read_client_request to serverRequest.operationX,
i.e. all functions in between (in server8.cpp) have been turned into coroutines.

cs3-client2 (client2.cpp) is a variant of cs3-client1 (client1.cpp).
cs3-client2 allows coroutine mainflow() to follow the progress of coroutine mainflow(process_info_t &process_info).
