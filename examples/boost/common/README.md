# Boost common operations (easy start implementation)

The following classes use Boost ASIO library functionality:
* class CommCore (files commcore.h and commcore.cpp) contains operations that are common to the client and server side: 
  read, write, start timers, closing, etc.
+ class CommClient (file commclient.h and commclient.cpp) implements the client side of an application. 
  CommClient inherits the functionality of CommCore and just adds the connect operation.
* class CommServer (file commserver.h and commserver.cpp) implements the server side of an application.
  Notice that CommServer does not inherit the functionality of CommCore; it just defines the accept operation.
  The communication with the connected client is then performed using a CommCore object.