# corolib - Boost

Files commclient.h, commcore.h and commserver.h use boost/asio and are only used by the Boost examples.

The following classes use Boost library functionality:
* class CommCore (files commcore.h and commcore.cpp) contains operations that are common to the client and server side: 
  read, write, start timers, closing, etc.
+ class CommClient (file commclient.h and commclient.cpp) implements the client side of an application. 
  CommClient inherits the functionality of CommCore and just adds the connect operation.
* class CommServer (file commserver.h and commserver.cpp) implements the server side of an application.
  CommServer inherits the functionality of CommCore and just add the accept operation.
  The communication with the connected client is then performed using a CommCore object.