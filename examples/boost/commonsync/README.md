# Boost common operations (synchronous implementation)

This library contains the definitions of class CommCore and CommClient in terms of (blocking) synchronous operations.

Each synchronous operation runs on a thread taken from a thread pool.

After the execution of the operation, a functor is placed on an event queue.
This functor will be popped from the event queue, usually in the main() function of the application, after which the functor is called.
The functor places the result of the synchronous operation (could be void) in the async_operation return object.
Then, it resumes the coroutine from where the library function was called.
