# Why use C++ coroutines for I/O intensive applications?

## Introduction

In this study we consider a file-like API with
* synchronous (blocking) functions create, open, write, read, close and remove, see file [sync.h](./sync.h)
* asynchronous (non-blocking) functions async\_create, async\_open, async\_write, async\_read, async\_close and async\_remove, see files [async.h](./async.h) and [asyncthr.h](./asyncthr.h)

The API functions do not perform file access; they just print a message
and do the minimum that is necessary to keep the application running.
For the asynchronous API, this minimum is "registrating" a callback function (completion handler) with an event queue.

This API is similar to that used for communication or networking with
* synchronous (blocking) functions connect, write, read, disconnect and wait.
* asynchronous (non-blocking) functions async\_connect, async\_write, async\_read, async\_disconnect and async\_wait.

The following table gives an overview of all example programs present in this directory:

| program                  | API        | event queue     | operations              | comment      |
| ------------------------ | ---------- | --------------- | ----------------------- | ------------ |
| p1000-sync.cpp           | sync.h     | (not present)   |                         | 6 operations |
| p1001-sync.cpp           | sync.h     | (not present)   |                         | 7 operations |
| p1010-sync-thread.cpp    | sync.h     | (not present)   |                         | 6 operations |
| p1011-sync-thread.cpp    | sync.h     | (not present)   |                         | 7 operations |
| p1020-async.cpp          | async.h    | eventqueue.h    |                         | 6 operations |
| p1021-async.cpp          | async.h    | eventqueue.h    |                         | 7 operations |
| p1025-async.cpp          | async.h    | eventqueue.h    |                         | 6 operations |
| p1026-async.cpp          | async.h    | eventqueue.h    |                         | 7 operations |
| p1030-async-thr.cpp      | asyncthr.h | eventqueuethr.h | operationsthr.h         | 6 operations |
| p1031-async-thr.cpp      | asyncthr.h | eventqueuethr.h | operationsthr.h         | 7 operations |
| p1040-coroutine.cpp      | async.h    | eventqueue.h    | operations-coroutine.h  | 6 operations |
| p1041-coroutine.cpp      | async.h    | eventqueue.h    | operations-coroutine.h  | 7 operations |
| p1042-coroutine.cpp      | async.h    | eventqueue.h    | operations-coroutine.h  | 7 operations |
| p1060-corolib.cpp        | async.h    | eventqueue.h    | operations-corolib.h    | 6 operations |
| p1061-corolib.cpp        | async.h    | eventqueue.h    | operations-corolib.h    | 7 operations |
| p1062-corolib.cpp        | async.h    | eventqueue.h    | operations-corolib.h    | 7 operations |
| p1070-corolib-thread.cpp | async.h    | eventqueuethr.h | operations-corolibthr.h | 6 operations |
| p1071-corolib-thread.cpp | async.h    | eventqueuethr.h | operations-corolibthr.h | 7 operations |
| p1072-corolib-thread.cpp | async.h    | eventqueuethr.h | operations-corolibthr.h | 7 operations |


## From synchronous to the use of coroutines

The initial program [p1000-sync.cpp](./p1000-sync.cpp) contains a function called synchronous() 
that calls the API functions in the order listed above.

These are the first lines of function synchronous():

```c++
void synchronous(int st)
{
    int i = st;
    print(PRI1, "synchronous: begin\n");

    print(PRI1, "synchronous: %d: create\n", st);
    i = create(i + 1);
    assert(i == st + 1);

	print(PRI1, "synchronous: %d: create complete: open\n", i);
    i = open(i + 1);
    assert(i == st + 2);

    print(PRI1, "synchronous: %d: open complete: write\n", i);
    i = write(i + 1);
    assert(i == st + 3);

// ...
```

The main program calls function synchronous() twice.
Because the API functions are blocking (if using a real implementation),
it takes twice the time to perform the two calls of synchronous() than just calling synchronous() once.
If the 2 calls of synchronous would access 2 different files,
then a lot of time is lost because the program is mainly waiting for I/O actions to complete.

The second program [p1001-sync.cpp](./p1001-sync.cpp) is a simple variant of the previous one 
that performs a second write after the read operation.

To gain performance, we can run the calls of synchronous() on a separate thread.
Because we assume there are no shared variables between both threads, no synchronization is necessary.

[p1010-sync-thread.cpp](./p1010-sync-thread.cpp) is the threaded variant of [p1000-sync.cpp](./p1000-sync.cpp) and
[p1011-sync-thread.cpp](./p1011-sync-thread.cpp) is the threaded variant of [p1001-sync.cpp](./p1001-sync.cpp)

Both threads consume resources (such as a stack) but are waiting most of the time for an I/O operation to complete.

Can we do better with an asynchronous API?

[p1020-async.cpp](./p1020-async.cpp) is the first attempt for an asynchronous variant.
Instead of a single synchronous() function, we now have 7 functions (the start function asynchronous()
and 6 on_XXX callback functions).
We also need a context object that is passed between the functions
and that contains the local variables and function parameters from the first examples.

In this simple example, the callback functions follow each other in a sequential order,
so we can still read the program from top to bottom.
For more complex flows, this may not be the case anymore.

The following code fragment shows the context struct and the first functions from [p1020-async.cpp](./p1020-async.cpp):

```c++
struct context_t
{
    int st;
    int i;
};

void asynchronous(int st) {
    print(PRI1, "asynchronous: begin\n");
    context_t* ctxt = new context_t{ st, st + 1 };
    async_create(on_create_complete, ctxt);
    print(PRI1, "asynchronous: end\n");
}

void on_create_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 1);
    print(PRI1, "on_create_complete(): %d: open\n", pctxt->i++);
    async_open(on_open_complete, ctxt);
}

void on_open_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 2);
    print(PRI1, "on_open_complete(): %d: write\n", pctxt->i++);
    async_write(on_write_complete, ctxt);
}

// ...
```

Notice that each callback function has to know the next step in the flow to start the next API function,
so these callback functions are "chained/stitched" together.
For example, on\_create\_complete() calls async\_open() with on\_open\_complete() as argument
and on\_open\_complete() calls async\_write() with on\_write\_complete() as argument.

[p1021-async.cpp](./p1021-async.cpp) is the counter part of [p1001-sync.cpp](./p1001-sync.cpp).
Because of the chaining just described, we need to duplicate the callback functions up till the point
where the flow between [p1000-sync.cpp](./p1000-sync.cpp) and [p1001-sync.cpp]./p1001-sync.cpp) differs,
i.e. until the latter program performs a second write after the read (instead of closing the file right away).
This explains the use of a 2 or 2a at the end of the on\_XXX callback function names.

This distinct naming is not necessary using these separate source code files,
but would be necessary in case [p1020-async.cpp](./p1020-async.cpp) and [p1021-async.cpp](./p1020-async.cpp) were merged.
An alternative is to pass an extra data member in the context struct that allows distinguishing
between the 6 step flow and the 7 step flow.
In case more variants are introduced, more variables will be necessary.
This will clutter the code because all variants are somehow coded in a single chain of callback fucntions.

[p1025-async.cpp](./p1025-async.cpp) uses lambdas in two variants.
The first variant (function asynchronous\_lambdas()) uses a single function with 7 lambdas inside.
The second variant (function asynchronous\_lambdas()) uses a chain of 7 functions.
This chain resembles the chain of functions in [p1020-async.cpp](./p1020-async.cpp).

The following are the first functions from [p1025-async.cpp](./p1025-async.cpp):

```c++
void asynchronous_lambdas2(int st) {
    print(PRI1, "asynchronous_lambdas: begin\n");
    start_create(st, st+1);
    print(PRI1, "asynchronous_lambdas: end\n");
}

void start_create(int st, int i) {
    async_create(
        [st, i]() {
            assert(i == st + 1);
            print(PRI1, "create complete: %d: open\n", i);
            start_open(st, i+1);
        });
}

void start_open(int st, int i) {
    async_open(
        [st, i]() {
            assert(i == st + 2);
            print(PRI1, "open complete: %d: write\n", i);
            start_write(st, i+1);
        });
}

// ...
```

Notice that these functions are still chained.
For exaample, start\_create() calls start\_open() and start\_open() calls start\_write().

[p1026-async.cpp](./p1026-async.cpp) is the counter part of [p1021-async.cpp](./p1021-async.cpp).

Can we do better with the asynchronous API, i.e. can we get rid of the hard-coded chaining
and return to the sequential style used in the synchronous() examples?

Yes, we can, but at the expense of running the asynchronous\_operations() function and the eventloop on a separate thread.

[p1030-async-thread.cpp](./p1030-async-thread.cpp) and [p1031-async-thread.cpp](./p1031-async-thread.cpp) illustrate this approach.
We can now place the functions in a separate file and combine them in the order that is appropriate for the application.

The following is code from [operationsthr.h](./operationsthr.h) with the application-independent functions

```c++
struct operations : public thread_awaiter {
    void start_create(int i) {
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "async_create handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_create handler: end\n");
            });
    }

    void start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        async_open([this, i]() {
            print(PRI1, "async_open handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_open handler: end\n");
            });
    }

    void start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        async_write([this, i]() {
            print(PRI1, "async_write handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_write handler: end\n");
            });
    }

// ...
```

and these functions can be used as follows:

```c++
void asynchronous_operations(int st) {
    int i = st;
    print(PRI1, "asynchronous: begin\n");

    operations ops;

    ops.start_create(i + 1);
    i = ops.get_result();
    assert(i == st + 1);

    print(PRI1, "asynchronous_operations: %d: create complete: open\n", i);
    ops.start_open(i + 1);
    i = ops.get_result();
    assert(i == st + 2);

    print(PRI1, "asynchronous_operations: %d: open complete: write\n", i);
    ops.start_write(i + 1);
    i = ops.get_result();
    assert(i == st + 3);

// ...
```

The event loop thread on which the callback/completion lambda runs on the one hand
and the thread(s) running function asynchronous\_operations() with an operations object on the other hand,
use a semaphore to synchronize.

The core functionality is implemented in class thread_awaiter:

```c++
class thread_awaiter
{
public:
    ~thread_awaiter() {
        print(PRI1, "thread_awaiter::~thread_awaiter(...)\n");
        m_result = -2;
    }

    void set_result_and_release(int result = 0) {
        print(PRI1, "thread_awaiter::set_result_and_release(%d)\n", result);
        m_result = result;
        m_sema.release();
    }

    int get_result() {
        m_sema.acquire();
        print(PRI1, "thread_awaiter::get_result(): returns %d\n", m_result);
        return m_result;
    }

protected:
    int m_result = -1;
    CSemaphore m_sema;
};
```

We have restored the original sequential flow, introduced higher-level API functions that are independent of each other
(i.e., there is no chaining anymore), but at the expense of introducing threads.

Can we get rid of these threads, but maintain the sequential flow while still using the asynchronous API?

Again, we can. Coroutines come to the rescue:

[p1040-coroutine.cpp](./p1040-coroutine.cpp), [p1041-coroutine.cpp](./p1041-coroutine.cpp)
and [p1042-coroutine.cpp](./p1042-coroutine.cpp) illustrate this approach
using a simple coroutine implementation.

The following is code from [operations-coroutine.h](./operations-coroutine.h) with the application-independent functions:

```c++
struct operations : public mini_awaiter {
    operations& start_create(int i) {
        m_ready = false;
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "async_create handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_create handler: end\n");
        });
        return *this;
    }

    operations& start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        m_ready = false;
        async_open([this, i]() {
            print(PRI1, "async_open handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_open handler: end\n");
        });
        return *this;
    }

    operations& start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        m_ready = false;
        async_write([this, i]() {
            print(PRI1, "async_write handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_write handler: end\n");
        });
        return *this;
    }

// ...
```

and these functions can be used as follows:

```c++
task coroutine_operations(int st) {
    int i = st;
    print(PRI1, "coroutine_operations2: begin\n");

    operations ops;

    print(PRI1, "coroutine_operations2: %d: create\n", i);
    i = co_await ops.start_create(i + 1);
    assert(i == st + 1);

    print(PRI1, "coroutine_operations2: %d: create complete: open\n", i);
    i = co_await ops.start_open(i + 1);
    assert(i == st + 2);

    print(PRI1, "coroutine_operations2: %d: open complete: write\n", i);
    i = co_await ops.start_write(i + 1);
    assert(i == st + 3);

// ...
```

The counter part of class thread\_awaiter (see above) is class mini\_awaiter:

```c++
class mini_awaiter
{
public:
    ~mini_awaiter() {
        m_result = -2;
    }

    bool await_ready() {
        return m_ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        m_awaiting = awaiting;
    }

    int await_resume() {
        return m_result;
    }

    void set_result_and_resume(int result = 0) {
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                m_awaiting.resume();
            }
            else {
                print(PRI1, "mini_awaiter::set_result_and_resume() did not resume() because m_awaiting.done() returned true\n");
            }
        }
        else {
            print(PRI1, "mini_awaiter::set_result_and_resume() could not resume() because m_awaiting == nullptr\n");
        }
    }

protected:
    std::coroutine_handle<> m_awaiting = nullptr;
    bool m_ready = false;
    int m_result = -1;
};
```

Function set\_result\_and\_resume(int result = 0) replaces set\_result\_and\_release(int result = 0).
In these function names, 'resume' refers to coroutine resume, while 'release' refers to semaphore release.

The 3 functions await\_ready(), await\_suspend() and await\_resume() (that are the lower-level functions of operator co\_await)
replace get\_result().

[p1060-corolib.cpp](./p1060-corolib.cpp), [p1061-corolib.cpp](./p1061-corolib.cpp) and [p1062-corolib.cpp](./p1062-corolib.cpp)
use corolib instead of the simple implementations in this directory.

The operationsCL (CL stands for corolib) class is now defined as follows:

```c++
class operationsCL : public CommService
{
public:
    async_operation<int> start_create(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index};
        async_create([this, index, i]() {
            print(PRI1, "async_create handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_create handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_open(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_open([this, index, i]() {
            print(PRI1, "async_open handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_open handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_write(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_write([this, index, i]() {
            print(PRI1, "async_write handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_write handler: end\n");
            });
        return ret;
    }

// ...
```

The application-level coroutine coroutine\_corolib() looks very much the same 
as the one in e.g. [p1040-coroutine.cpp](./p1040-coroutine.cpp):

```c++
async_task<void> coroutine_corolib(int st) {
    int i = st;
    print(PRI1, "coroutine_corolib: begin\n");

    operationsCL ops;

    print(PRI1, "coroutine_corolib: %d: create\n", i);
    i = co_await ops.start_create(i + 1);
    assert(i == st + 1);

    print(PRI1, "coroutine_corolib: %d: create complete: open\n", i);
    i = co_await ops.start_open(i + 1);
    assert(i == st + 2);

    print(PRI1, "coroutine_corolib: %d: open complete: write\n", i);
    i = co_await ops.start_write(i + 1);
    assert(i == st + 3);

// ...
```

We can also combine coroutines with threads.
[p1070-corolib-thread.cpp](./p1070-corolib-thread.cpp), [p1071-corolib-thread.cpp](./p1071-corolib-thread.cpp)
and [p1072-corolib-thread.cpp](./p1072-corolib-thread.cpp) illustrate this apprach.
These main() function in the p107X series resembles the main() function in the p103X series,
while the application-level coroutines are the same (although there is a difference in the implementation).

Although these examples run fine, they introduce unnecessary complexity (in the use of threads).

## Summary

The following table illustrates the different approaches (styles) that were discussed in the previous section:

| "paradigm"    | single-threaded                    | event loop runs on dedicated thread         |
| ------------- | ---------------------------------- | ------------------------------------------- |
| synchronous   | (1) sequential style, not reactive | (2) sequential style, reactive              |
| asynchronous  | (3) fragmented stype, reactive     | (4) sequential style, reactive              |
| coroutines    | (5) sequential style, reactive     | (6) sequential style, reactive, but no need |

Style (1) is illustrated by [p1000-sync.cpp](./p1000-sync.cpp) and [p1001-sync.cpp](./p1001-sync.cpp).

Style (2) is illustrated by [p1010-sync-thread.cpp](./p1010-sync-thread.cpp) and [p1011-sync-thread.cpp](./p1011-sync-thread.cpp).

Style (3) is illustrated by [p1020-async.cpp](./p1020-async.cpp) and [p1021-async.cpp](./p1021-async.cpp) and by
[p1025-async.cpp](./p1025-sync.cpp) and [p1026-async.cpp](./p1026-async.cpp).

Style (4) is illustrated by [p1030-async-thread.cpp](./p1030-async-thread.cpp) and [p1031-async-thread.cpp](./p1031-async-thread.cpp).

Style (5) is illustrated by [p1040-coroutine.cpp](./p1040-coroutine.cpp), [p1041-coroutine.cpp](./p1041-coroutine.cpp) and 
[p1042-coroutine.cpp](./p1042-coroutine.cpp) and 
by [p1060-corolib.cpp](./p1060-corolib.cpp), [p1061-corolib.cpp](./p1061-corolib.cpp) and [p1062-corolib.cpp](./p1062-corolib.cpp).

Style (6) is illustrated by [p1070-corolib-thread.cpp](./p1070-corolib-thread.cpp), [p1071-corolib-thread.cpp](./p1071-corolib-thread.cpp) and
[p1072-corolib-thread.cpp](./p1072-corolib-thread.cpp).

Style (5) is the preferred style if coroutines are available.
If this is not the case, style (4) comes closest to the coroutine style and could be used as a 
step towards the possible or eventual adoption of coroutines.

## More complex examples

The file-like APIs used in the examples in this directory do not have a real implementation.

The Boost ASIO functions async\_connect, async\_write, async\_read\_until, async\_wait come closest to the dummy functions used here.

A style (3) example can be found in [client1.cpp](../../examples/boost/clientserver0/client1.cpp).

Style (4) examples can be found in [client2.cpp](../../examples/boost/clientserver0/client2.cpp) and
[client3.cpp](../../examples/boost/clientserver0/client3.cpp).

Style (5) examples are present in directories [clientserver1](../../examples/boost/clientserver1),
[clientserver2](../../examples/boost/clientserver2), [clientserver3](../../examples/boost/clientserver3) and
[clientserver4](../../examples/boost/clientserver4).
