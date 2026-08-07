# Client applications using a synchronous API

This folder contains a subset of applications from ../clientserver1.
These applications have been simplified and adapted for use with the library in ../commonsync.

A major difference is the use of an event queue (name myeventqueue, type CommQueue).

Instead of processing events using

```c++
    ioContext.run();
```

events are now processed from the event queue using

```c++
    runEventQueue(myeventqueue);
```

Despite the fact that the library runs each synchronous API call on a dedicated thread,
all the application code runs on a single thread!

After building, launch the applications in the following order:

* 'cs1-server' and 'cs1-clientserver' (from ../clientserver1) (in any order) and
* 'cs1s-client0' (one or more instances) and/or
* 'cs1s-client1' (one or more instances) and/or
* 'cs1s-client3WA' (one or more instances)

Alternatively:

* 'cs0-server' or 'cs0-server1' (from ../clientserver0) and
* 'cs1s-client0' or
* 'cs1s-client1' or
* 'cs1s-client3WA'
