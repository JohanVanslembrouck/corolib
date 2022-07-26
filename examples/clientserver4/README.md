# Two tier client-server application with ROS-like cancellable actions

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs4-server' is action server application with 'cs4-client1' as its client application.

After building, launch the applications in the following order:

* 'cs4-server' or 'cs4-server2'
* 'cs4-client1' (one or more instances)

This example has been inspired by actions in ROS (Robot Operating System):
- The client sends a goal to the action server.
- The action server sends a number of feedback messages to the client, indicating the progress of the action.
- Finally, the action server sends the result to the client.

In this example, the action server sends 10 feedback messages to the client, once every second, followed by the result message (again after a delay of a second).

The client preforms N actions and it waits for 5 or 20 seconds (alternatingly) for the action to complete.
In the case the client waits only 5 seconds (action not yet complete), the client sends a stop request to the action server to terminate the action.
In the case of 20 seconds, the action will have completed, the client stops the timer and it does not send a stop request.

cs4-server2 is a variant of cs4-server: waiting for the write action to finish is done using the wait_any_awaitable object that is used 
to wait for the timer to expire or for the read action to complete.