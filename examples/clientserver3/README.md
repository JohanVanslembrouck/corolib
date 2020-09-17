# Two tier client-server application with layer of abstraction

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs3-server', 'cs3-server2', 'cs3-server3', 'cs3-server4' and 'cs3-server4' are server applications with 'cs3-client1' or 'cs3-client2' as their client application.

After building, launch the applications in the following order:

* 'cs3-server' or 'cs3-server2' or 'cs3-server3' or 'cs3-server4' or 'cs3-server5'
* 'cs3-client1' (one or more instances) or 'cs3-client2' (one or more instances)

In this application the API to the user does not use strings directly: a tiny abstraction layer has been added so that the application can use user-defined types that (in a real-life application) have to be marshalled onto or unmarshalled from \n-terminated strings.

cs3-server and cs3-server2 are equivalent. The difference is that server.cpp is self-contained, while server2.cpp has to include serverrequest.h and dispatcher.h.

cs3-server4 is an extension of cs3-server3. It uses a different Dispatcher implementation than cs3-server3 and cs3-server2. After having dispatched an operation (passed as a lambda), the dispatcher also completes an asynchronous operation. This way main_one_client gets feedback on the invoked operation and can (if applicable) take some additional actions.

cs3-server5 allows coroutine mainflow_one_client to follow the progress of coroutine read_client_request.

cs3-client2 allows coroutine mainflow() to follow the progress of coroutine mainflow(process_info_t &process_info).
