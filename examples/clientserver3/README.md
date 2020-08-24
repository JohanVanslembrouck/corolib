# Two tier client-server application with layer of abstraction

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs3-server' and 'cs3-server2' are server applications with 'cs2-client1' as their client application.

After building, launch the applications in the following order:

* 'cs3-server' or 'cs3-server2'
* 'cs3-client1' (one or more instances)

In this application the API to the user does not use strings directly: a tiny abstraction layer has been added so that the application can use user-defined types that (in a real-life application) have to be marshalled onto or unmarshalled from \n-terminated strings.

cs3-server and cs3-server2 are equivalent. The difference is that server.cpp is self-contained, while server2.cpp has to include serverrequest.h and dispatcher.h 