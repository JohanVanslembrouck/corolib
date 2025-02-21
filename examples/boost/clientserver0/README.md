# Two tier client-server application without coroutines

The applications in this directory do not use coroutines.
They have been based upon [p0700c.cpp](../../../studies/corolab/p0700c.cpp),
[p0710c.cpp](../../../studies/corolab/p0710c.cpp) 
and [p0700s.cpp](../../../studies/corolab/p0700s.cpp) in [studies/corolab](../../../studies/corolab).

In contrast to the applications in [studies/corolab](../../../studies/corolab),
the applications in this directory use as much as possible lambdas instead of callback functions (std::bind).
The classes in the [common](../common) directory that implement coroutines also use lambdas.

The following is an overview of the applications:

* [server.cpp](./server.cpp) is a simple echo server application. It uses a dedicated thread to process client requests
  but because of the use of std::this_thread::sleep_for(), the delay in sleep_for() prevents multiple clients to be served
  in an interleaved way.

* [server1.cpp](./server1.cpp) is a variant of [server.cpp](./server.cpp) that uses async_wait() instead
  of std::this_thread::sleep_for(). This allows multiple clients to be served in an interleaved way.
  This is the preferred variant.

* [client0.cpp](./client0.cpp) uses the synchronous API functions connect(), write(), read() and close().
  This application uses a mainflow() function that can be read sequentially
  and that also executes the steps connect - write - read - close in a sequential way.

* [client1.cpp](./client1.cpp) uses the asynchronous API functions async_connect(), async_write() and async_read_until().
  In addition, async_wait() is used to to guard deadlines.
  All functions take a lambda that is passed as completion handler.
  The completion handler of function N starts function N+1. This is illustrated in the following table:

| step | start_ function  | calls            | completion handler calls |
| ---- | ---------------- | ---------------- | ------------------------ |
| 1    | start_connecting | async_connect    | start_writing            |
| 2    | start_writing    | async_write      | start_reading            |
| 3    | start_reading    | async_read_until | stop                     |

  In this way, all functions are somehow chained/stitched together by the code in the completion handler.
  Inserting an extra step involves modifying the code of the completion handler that preceeds the new step
  and adding the removed code to the completion handler of the new step.

  There is no mainflow() function anymore: it has been replaced with the chain of functions shown in the table above. 

* [client2.cpp](./client2.cpp) makes the completion handler functions independent of the next step
  and allows re-introducting the mainloop() function. Yet, this implementation uses the asynchronous API.
  This has been accomplished by running the event loop (ioContext.run()) on a separate thread.
  The completion handlers run on that thread and release a semaphore that was acquired from the mainflow().
  This allows restoring the original sequential flow.

  client2.cpp calls the mainflow() function 3 times in main(). These calls run sequentially.

* [client3.cpp](./client3.cpp) is a variant of client2.cpp, running 3 calls of mainflow() on a separate thread
  instead of sequentially. Only the first instance of mainflow() will start the event loop (again on a separate thread).
  The two other instances will be served from the same event loop. To make sure that the event loop has started,
  there is a small delay after launching the first thread that runs the mainflow() that starts the event loop.

Note that Boost ASIO applications are usually single-threaded in the sense that the async_ functions
and the event loop (ioContext.run()) - and consequently also the completion handlers - are run on the same thread,
see [client1.cpp](./client1.cpp).
The cs0-client2 and cs0-client3 executables sometimes hang: a possible reason is that Boost ASIO is not
thread-safe (i.e., it was not the intention to use it in this multi-threaded way).

After building, launch the applications in the following order:

* 'cs0-server' or (preferably) 'cs0-server1'
* 'cs0-client0' or 'cs0-client1' or 'cs0-client2' or 'cs0-client3'

Note that [client3.cpp](./client3.cpp) is not that far away from the use of coroutines,
see the sibling directories [clientserver1](../clientserver1), [clientserver2](../clientserver2),
[clientserver3](../clientserver3) and [clientserver4](../clientserver4). Roughly stated:

1. Instead of running mainflow() calls on a different thread, mainflow() will become a coroutine,
   replacing preemptive multi-tasking provided by threads with cooperative multi-tasking provided by coroutines.
2. Semaphores will be replaced with the suspend/resume mechanism of coroutines:
   * semaphore acquire will be replaced with co_await
   * semaphore release will be replaced with a resume of the coroutine
