# Study final_suspend(): avoiding memory leaks

## Introduction

There are lots of things to learn when you are new to C++ coroutines, especially when you want to develop a coroutine library yourself:

* co_return, co_await, co_yield,
* concepts (not in the sense of C++ concepts) such as 
  * coroutine state or frame,
  * coroutine type,
  * continuation,
  * eager and lazy start coroutines,
  * awaitable and awaiter types (with its 3 functions await_ready(), await_suspend() and await_resume()),
* suspend_always and suspend_never,
* promise_type, with its many functions
  * get_return_object(),
  * initial_suspend(),
  * final_suspend(), 
  * return_value(), return_void(), yield_value(),
  * unhandled_exception(),
  * get_return_object_on_allocation_failure().

Starting from simple implementations, you can make them more complete (and complex).

A simple implementation often uses the following definitions for initial_suspend() and final_suspend():

```c++
class task {
public:
    class promise_type {
    public:
        // ...

        std::suspend_never initial_suspend() noexcept {  // eager start coroutine
            return {};
        }

        std::suspend_always final_suspend() noexcept {
            return {};
        }

        // ...
    };
};
```

The reader is referred to ../study-initial_suspend for a comparison of eager and lazy start corouutines.

This text explores final_suspend().

## final_suspend() returns a standard awaiter type

In this case, final_suspend() returns std::suspend_always or std::suspend_never.

### final_suspend() returns std::suspend_always

The advantage of final_suspend() returning std::suspend_always is that the results 
expected from your coroutine-based applications are usually correct (if there are no logical errors in the application code).
The disadvantage is the same.

Unfortunately, some of these behaviorly correct coroutine applications may not be able to destroy all coroutine frames
(with their "embedded" promise_type objects), leading to memory leaks because coroutine frames are typically allocated on the heap.
It is easy to stick to this easy implementation and to overlook these memory leaks,
which may only lead to problems in long-running applications.

The following definition of the task destructor cannot prevent the memory leaks:

```c++
class task {
public:
    // ...

    ~task() {
        if (coro_)
            if (coro_.done()) {     // Do not destroy the coroutine frame if it has not yet reached the final suspend point
                coro_.destroy();
                coro_ = {};
            }
    }
```

When a coroutine has not yet reached its final suspend point,
we cannot destroy the coroutine frame when the task object 
(containing the coroutine_handle coro_ to the coroutine frame) goes out of scope.

When it reaches the final suspend point, the coroutine remains suspended,
because there is no code that will resume the coroutine (so that it will finally release its coroutine frame).

The examples p10X0_sa.cpp (lazy start coroutines) and p10X0e_sa.cpp (eager start coroutines) show that the results are correct,
but that the applications that complete asynchronously do not destroy 3 of the 4 promise_type objects. "sa" stand for suspend_always.

```
00: coroutine1(): co_return 4;
00: main(): int v = a.get_result();
00: main(): v = 4;
00: main(): return 0;
00: 00000008DD39FDC8: task::~task()
00: 0000010B8D1910C0: promise_type::~promise_type()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       1       3       4
00: --------------------------------------------------------
```

Here, cons stands for the number of objects constructed, dest for the number of objects destructed, diff for cons - dest (should be 0!),
max for the maximum number of objects alive at any time.
There are two types of objects: cor or coroutine objects (class "task" in this case) and pro or promise_type objects.

### final_suspend() returns std::suspend_never

So, what would happen if final_suspend() returns std::suspend_never instead of std::suspend_always?
This is easy to implement.

The examples p11X0_sn.cpp (lazy start coroutines) and p11X0e_sn.cpp (eager start coroutines) show that the results are *incorrect*,
but that all promise_type objects are released. X = 0, 1, 2. "sn" stand for suspend_never.

```
00: main(): int v = a.get_result();
00: main(): v = -572662307 = dddddddd;
00: main(): return 0;
00: 000000669F2FF888: task::~task()
00: 000000669F2FF888: task::~task(): !coro.done()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       4       0       4
00: --------------------------------------------------------
```

The reason is that we read the results from a promise_type object in a coroutine frame that has already been deallocated.
The operating system has filled the memory with 0xdddddddd on Windows.
Note that on Ubuntu 22.04, the result is correct, because the memory has not been re-initialized.

### final_suspend() returns std::suspend_never with result stored in the task object

Instead of storing the result in the promise_type object, the previous section seems to indicate that the problem
can be solved by storing the result in the task object, so that we can fetch the result from there.

The examples p11X0_sn.cpp (lazy start coroutines) and p11X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released. X = 5, 6, 7. Subtract 5 to get the corresponding example from the previous section.

## final_suspend() returns a custom final_awaiter type

The following sections explore the behavior of custom final_awaiter types 
defining their own await_ready(), await_suspend() and await_resume() functions.
The function await_suspend() can have three return types: void, bool and coroutine_handle<>.

### await_suspend() returns void

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() return void.

The examples p12X0_sn.cpp (lazy start coroutines) and p12X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released.

### await_suspend() returns bool

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() returns bool.

The examples p13X0_sn.cpp (lazy start coroutines) and p13X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released. 

### await_suspend() returns std::coroutine_handle<>

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() return std::coroutine_handle<>.

The examples p14X0_sn.cpp (lazy start coroutines) and p14X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released.

## Using corolib async_task and async_ltask

The examples p15X0_sn.cpp (lazy start coroutines using async_ltask) and p15X0e_sn.cpp (eager start coroutines using async_task)
show that the results are correct and that all promise_type objects are released.
