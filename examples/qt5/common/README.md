# Common classes for use with the Qt examples

Notice that only tcpclientco.h|.cpp and tcpclientco1.h|.cpp use coroutine functionality.
All other classes are independent of C++ coroutines.
In other words, only the client classes support coroutine functionality (at this moment).

The implementation in tcpclientco.h|.cpp is closer to the Boost way of working:
with every asynchronous operation that is started, a callback function is passed to the framework.
Once called, the Boost framework "forgets" the callback.
However, this is not the way Qt works: a slot function remains connected with a signal until it is explicitly disconnected.
When imitating the Boost way of working, this means that the slot function has to disconnect itself from the signal.
If not, the number of slot functions connected to the same signal will grow with every asynchronous operation invocation
that makes a new connect.

For example, when using timers, the timer slot function will be called N times if a timer was
started N times. If the slot function starts the timer again, this would lead to an exponential growth in the number of timers that is started and that expires.

The implementation in tcpclientco1.h|.cpp follows the Qt way of working.
Slot functions are connected to a signal only once, at startup or just before the point where they are needed.
