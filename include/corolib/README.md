# corolib

The library has two parts:
* Files async_operation.h, async_task.h, auto_reset_event.h, commservice.h, oneway_task.h, print.h, semaphore.h, wait_all_awaitable.h, wait_all_counter.h, wait_any.h and wait_any_awaitable.h are independent of Boost and are also used by the Qt examples.
* Files commclient.h, commcore.h and commserver.h use boost/asio and are only used by the Boost examples.
Folder lib contains the source (.cpp) files for the header files (.h) in include/corolib.

The following classes are related:
* template<typename TYPE> class async_operation (file async_operation.h) implements operator co_await() but it does not define a promise_type.
  Therefore it cannot be used to create coroutines.
  This class is meant to be used as a return type of a normal C++ (member) function, making the bridge between non-coroutines and coroutines.
  Objects of this class will be co_awaited in coroutines created with class async_task or one_task, see next points.
  The class is derived from class async_operation_base.
  This class has a pointer to class CommService described below. 
* template<typename TYPE> struct async_task (file async_task.h) implements both operator co_await() and a promise_type. Its name refers to the coroutine type 'task' in C#.
  This type allows defining coroutines, i.e. functions that use co_await and co_return in their implementation and have async_task as return type.
  promise_type::initial_suspend returns suspend_never, while promise_type::final_suspend returns suspend_always.
* struct oneway_task (file one_task.h) defines a promise_type but does not implement operator co_await().
  It can be used to define coroutines that cannot be awaited upon.
  Both promise_type::initial_suspend and promise_type::final_suspend return suspend_never.
* struct auto_reset_event (file auto_reset_event.h) implements operator co_await() but it does not define a promise_type.
  It could be considered a stripped-down version of class async_operation.
  This class can be used as a return type of a normal C++ (member) function.
  The returned object is then co_awaited in a coroutine. 
  On resumption in co_await, its state is reset automatically, so when the same object is co_awaited again, 
  the co_await call will suspend until the object's resume() function is called again.
  The auto_reset_event class could be considered to be a coroutine-style implementation of a semaphore. 
  Note that async_operation allows to reset the state on demand using the function reset().
  
The following class is the base class for all asynchronous operation classes we want to implement.

- class CommService (file commservice.h) is the base class for all classes defining functions that return an object of type class async_operation.
  The class defines an array of pointers to async_operations. 
  An index to the array (used as circular buffer) is passed from the function that initiates the asynchronous operation, to the completion handler 
  (a lambda that is used as callback function).
  The address of the async_operation_base (or a derived class) object cannot be used as argument in the lambda closure.
  The constructor of the async_operation_base object will place its address in the array at the given index.

The following classes use the Boost library:
- class CommCore (files commcore.h and lib/commcore.cpp) contains operations that are common to the client and server side: read, write, start timers, closing, etc.
- class CommClient (file commclient.h and lib/commclient.cpp) implements the client side of an application. 
  CommClient inherits the functionality of CommCore and just adds the connect operation.
- class CommServer (file commserver.h and lib/commserver.cpp) implements the server side of an application.
  CommServer inherits the functionality of CommCore and just add the accept operation.
  The communication with the connected client is then performed using a CommCore object.

The following classes are related and deal with the completion of one or all operations.
- template<typename TYPE> struct wait_all_awaitable (file wait_all_awaitable.h) implements operator co_await.
  Its contructor is passed a list of operations that all have to be completed before the coroutine will resume.
  All operations receive a pointer to the wait_all_counter data member (see next point).
  Each operation calls the wait_all_counter's completed() function when it completes.
- struct wait_all_counter (file wait_all_counter.h) is used as data member in wait_all_awaitable.
  It implements a counter that is decremented each time its completed() function is called.
  When the counter reaches zero, the coroutine is resumed.
- template<typename TYPE> struct wait_any_awaitable (file wait_any_awaitable.h) implements operator co_await.
  Its contructor is passed a list of operations; one of the operations has to be completed before the coroutine will resume.
  Each operation receives a pointer to a dedicated wait_any object (see next point).
  Each operation calls the wait_all_counter's completed() function when it completes.
- struct wait_any (file wait_any.h) is used as a data member in wait_any_awaitable 
  (more in particular for a data member with type std::vector<wait_any*>).
  When an operation calls the completed() function on its wait_any object, the coroutine is resumed.
  
Other:
- class Semaphore (file semaphore.h) is the implementation of a semaphore using std::mutex and std::condition_variable.
- function print (files print.h and lib/print.cpp) allows printing messages with several priorities.
