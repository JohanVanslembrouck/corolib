# Two tier client-server application with cancellable actions

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs2-server' and 'cs2-server2' are server applications with 'cs2-client1' as their client application.

After building, launch the applications in the following order:

* 'cs2-server' or 'cs2-server2
* 'cs2-client1' (one or more instances) or 'cs2-client1a' (one or more instances)

The client1 application writes a string to the server.
Coroutine start_reading_timed starts a read action and a timer.
It then awaits the reply from the server or the timer expiry.
In the case the client receives a reply, it sends an "ACK" message to the server.
In the case of timer expiry, the client sends a "STOP" request to the server to stop the running action.

In contrast with client1.cpp, the implementation of start_reading_timed in client1a.cpp uses a loop.
The behavior should be identical.

The server application receives a string from the client.
It starts a timer to simulate a delay during the calculation of the result.
It awaits the expiry of the timer or the reception of the "STOP" request from the client.
In the case of timer expiry, the server sends the reply to the client.
In the case of a "STOP" request, it sends an "ACK" to the client.

The client1a application has the same behaviour as the client1 application, but it uses a small loop to have a somewhat smaller program.

The server2 application has the same behaviour as the server application.
It is more complicated  because it has split the one_client coroutine of the server application
into two coroutines: one_client and an auxiliary coroutine one_client_write_reply.
Both coroutines have to coordinate their actions, using two asynchronous operations: cancelAction and completeAction.
