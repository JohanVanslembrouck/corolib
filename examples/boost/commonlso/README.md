# Boost common operations (lazy start variant)

In contrast to the eager start variant, the 5 operations (accept, connect, read, write, timing) are defined in
dedicated classes.
The definition of these operations has been based on the definition of operations in cppcoro
(https://github.com/lewissbaker/cppcoro, https://github.com/andreasbuhr/cppcoro).

The following classes use Boost ASIO library functionality:
* class CommCore (files commcorelso.h and commcorelso.cpp) contains operations that are common to the client and server side: 
  read, write, start timers, closing, etc.
+ class CommClient (file commclientlso.h and commclientlso.cpp) implements the client side of an application. 
  CommClient inherits the functionality of CommCore and just adds the connect operation.
* class CommServer (file commserverlso.h and commserverlso.cpp) implements the server side of an application.
  Notice that CommServer does not inherit the functionality of CommCore; it just defines the accept operation.
  The communication with the connected client is then performed using a CommCore object.
