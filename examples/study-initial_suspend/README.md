# Study initial_suspend(): lazy or eager start?

## Introduction

The examples in this folder are heavily inspired by the article
https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
and the corresponding code in https://godbolt.org/z/-Kw6Nf, https://godbolt.org/z/gy5Q8q, https://godbolt.org/z/7fm8Za 
and https://godbolt.org/z/9baieF

The main purpose of this study is to compare the advantages and disadvantages of
lazy start and eager start coroutines for the three variants of await_suspend(),
and the impact this choice has on the behavior of the application.

## Lazy versus eager start

In a coroutine with lazy start, initial_suspend() returns std::suspend_always;
in a coroutine with eager start, initial_suspend() returns std::suspend_never.

await_suspend() can have one of the following three return types: void, bool, or coroutine_handle<>.

### Lazy start

The following provides an abstraction of the code that can be found in the files
task_void.h, task_bool.h and task_coroutine_handle.h.

    task::promise_type::initial_suspend() returns std::suspend_always

    task::awaiter
        await_ready() returns bool
	       return false;
        await_suspend(std::coroutine_handle<promise_type> h) returns
	       void                     // see task_void.h
	       bool                     // see task_bool.h
	       std::coroutine_handle<>  // see task_coroutine_handle.h
       await_resume() returns void

    task::promise_type::final_awaiter
        await_ready() returns bool
	        retur false;
        await_suspend(std::coroutine_handle<promise_type> h) returns
	        void                    // see task_void.h
	        bool                    // see task_bool.h
	        std::coroutine_handle<> // see task_coroutine_handle.h
        await_resume() returns void

    task::promise_type::final_suspend() returns std::final_awaiter

For all three return types, task::awaiter::await_suspend() has to resume coro_.
In case of void and bool return types, this is accomplished by the statement coro_.resume();
in case of a std::coroutine_handle<> return type, await_suspend has to return coro_.
The generated C++ code will then call coro_.resume().

All await_ready() implementations return false.
This function can be considered obsolete in the case of lazy start coroutines:
the coroutine will always suspend at a co_await statement and it will have to be resumed,
often from a co_await statement in another coroutine.

Notice that any variant of task::awaiter::await_suspend can be combined with any variant of
task::promise_type::final_awaiter::await_suspend, leading to nine possible combinations.

### Eager start

The following provides an abstraction of the code that can be found in the files
taske_void.h, taske_bool.h and taske_coroutine_handle.h.

    task::promise_type::initial_suspend() returns std::suspend_never

    task::awaiter
        await_ready() returns bool
	        return coro_.done(); (or return ready; if available)
        await_suspend(std::coroutine_handle<promise_type> h) returns
	        void                        // see taske_void.h
	        bool                        // see taske_bool.h
	        std::coroutine_handle<>     // see taske_coroutine_handle.h
        await_resume() returns void

    task::promise_type::final_awaiter
        await_ready() returns bool
	        return false;
        await_suspend(std::coroutine_handle<promise_type> h) returns
	       void                         // see taske_void.h
	       bool                         // see taske_bool.h
	       std::coroutine_handle<>      // see taske_coroutine_handle.h
        await_resume() returns void

    task::promise_type::final_suspend() returns std::final_awaiter

For none of the three return types, task::awaiter::await_suspend() has to resume coro_.
In case of a std::coroutine_handle<> return type, await_suspend has to return std::noop_coroutine().

Notice again that any variant of task::awaiter::await_suspend can be combined with any variant of
task::promise_type::final_awaiter::await_suspend, leading to nine possible combinations.

## Semantic issues with lazy start

Lazy start coroutines seem to alter the meaning of "co_await coroutine1(...)".
co_await should suspend a coroutine if the called coroutine (coroutine1 in this case) cannot provide the reply to its caller.
In case of lazy start coroutines, co_await seems to mean "to start the called coroutine" instead,
thus altering its intuitive meaning (semantics).

When coroutine1 calls coroutine2, which calls coroutine3, which calls coroutine4, etc.,
then suspend-resume operations will also be used on the calling path,
and not only on the return path (in case a coroutine cannot provide the reply to its caller).
There will be many more suspend-resume operations than in case of eager start coroutines.

Python and C# async/await use the intuitive eager variant.

## Lazy start may lead to mutually recursive calls when a coroutine is called in a loop

The following is the core example of the article:

```c++
task completes_synchronously() {
  co_return;
}

task loop_synchronously(int count) {
  for (int i = 0; i < count; ++i) {
    co_await completes_synchronously();
  }
}
```

This code will translate into mutual recursive calls when the await_suspend() functions return void or coroutine_handle<>:
loop_synchronously(int count) and completes_synchronously() call each other recursively.

With the împlementation of the task class in https://godbolt.org/z/-Kw6Nf, 
which is repeated here in p1000_void.cpp and in task_void.h,
the application crashes for "large" values of count.

The reason for this mutual recursion is the use of a lazy start coroutine type and not so much the fact that 
task::promise_type::final_awaiter::await_suspend(coroutine_handle<promise_type>) and
task::awaiter::await_suspend(coroutine_handle<> ) return void.

Indeed, by using an eager start coroutine type instead,
as is done in taske_void.h, both coroutines do not call each other recursively anymore.
Instead, the control flow is the same as that of normal functions: the coroutines do not suspend and resume,
but just use call and return.

## Eager start coroutines can be started automatically from main()

Eager start coroutines can be called and started from normal functions, in particular main().
The reason is that you don't have to co_await such a coroutine to resume them beyond the initial suspend point.

The code that comes with the mentioned article introduces two additional classes,
manual_executor and sync_wait_task to overcome the "impedance mismatch"
between a normal function (that cannot contain co_await) and a lazy start coroutine.
Notice that sync_wait_task is a coroutine type that uses eager start.

A coroutine using async_ltask from corolib (where ltask stands for lazy task) can be started from a normal function
(in particular main()) using the start() function:

```c++
async_ltask<int> a = obj.coroutine1();
a.start();
```

## Completion on another thread

Let's use the following example:

```c++
task foo()
{
    task t = start_operation1();
    co_await t;
    co_return;
}
```

foo() starts an asynchronous operation that may complete immediately (synchronously) or later on another thread (asynchronously).

In case of immediate completion, foo() does not suspend at the co_await statement and runs to completion on the same thread;
otherwise, foo() does suspend at the co_await statement and its completion will run on a different thread.

```c++
task bar()
{
   	task t = foo();
    // Point 1
    // Pre co_await code
    co_await t;         
    // Post co_await code
    co_return;
}
```

Point 1 is reached when foo() either suspends (or has completed synchronously):

* Lazy start: the suspend point in foo() will be the initial suspend point.

* Eager start: the suspend point in foo() is at the co_await statement in the function's body.

At the co_await statement:

* Lazy start: foo() will be resumed and enter its body. bar() suspends and returns control to its caller.

* Eager start:

  * foo() has completed synchronously: bar() will not suspend and run to completion.
 
  * foo() completes on another thread: there is a race condition between the original thread checking if foo() has already completed 
    and the other thread on which the completion of foo() takes place.

In case of a lazy start or an eager start with foo() not yet completed,
bar() suspends and returns control to its caller, that will return control to its caller, etc.
At some point, however, we will reach a point where we cannot "descend" any further,
otherwise we would "fall off" the original thread.
I call this point the "waiting point."

At any time, foo() may complete on another thread.
This is even possible while the original thread is still descending towards its waiting point.
To make things a bit easier, I will assume that the original thread reaches its waiting point
before foo() is completed.

Note that corolib provides a class, ThreadAwaker, that is used to allow the original thread to signal
to one or more completion threads that they may proceed.
The original thread will do this at the waiting point.

The following description is applicable to both lazy and eager start coroutines.

At the waiting point, the original thread must wait until foo() has completed
to either exit the application or to start waiting for new imput.
Alternatively, the application can start waiting for new imput and process this input even if foo() has not yet completed.
I will only describe the last case.

When foo() completes, the completion thread may run concurrently
with the original thread that is already processing a new event: the "Post co_await code" in bar() will run on the thread 
on which foo() is completed.
If this code manipulates any data that is also accessed while processing the new event, data has to
protected against concurrent access.
 
In other words: although the use of lazy start coroutines avoids the need for thread synchronization
as early as eager start coroutines does,
thread synchronization issues cannot be avoided when the completion thread runs concurrently with the original thread.

## task object of foo() goes out of scope

```c++
task bar()
{
    task t = foo();
    // Point 1
    // Pre co_await code
    // co_await t;         
    // Post co_await code
    co_return;
}
```

The co_await statement has been commented out.
At the end of bar(), task object t goes out of scope and it will destroy foo()'s coroutine frame.

Let's compare lazy and eager start.

* Lazy start: foo() has suspended at its initial suspend point. The task destructor can safely destroy foo()'s coroutine frame
  because foo() has not entered its body (and started an operation that has to be completed).

* Eager start(): foo() may (will) have started an operation that has not yet completed.
  The task desctructor must check if foo() is done().
  If so, the coroutine frame can be deleted, otherwise not. This can be immplemented as follows:

```c++
    ~task() {
        if (coro_)
            if (coro_done())
                coro_.destroy();
    }
```

In case of an eager start, foo() will eventually complete although the task object has gone out of scope.
The application returns a wrong result and not all coroutine frames and the contained promise objects will be deleted.

However, the code is wrong and must be corrected: the co_await statement must be present!

## Exception in Pre co_await code

```c++
task bar()
{
    task t = foo();
    // Point 1
    // Pre co_await code can raise an exception
    co_await t;         
    // Post co_await code
    co_return;
}
```

Let's compare lazy and eager start.

* Lazy start: foo() has suspended at its initial suspend point. If the exception is not caught,
  the task destructor can safely destroy foo()'s coroutine frame
  because foo() has not entered its body (and started an operation that has to be completed).

* Eager start: foo() may (will) have started an operation that has not yet completed. The co_await statement will be skipped.
  The task destructor must check if foo() is done(), see code fragment above. If so, the coroutine frame can be deleted, otherwise not.

In case of an eager start, foo() will eventually complete although the task object has disappeared.

Again, the code is wrong: the exception in the Pre co_await code must be caught and handled properly,
so that the co_await statement will be executed.

## Overview of the examples

p0000_void.cpp, p0100_void.cpp, p0110_bool.cpp and p0120_coroutine_handle.cpp correpond to 
https://godbolt.org/z/-Kw6Nf, https://godbolt.org/z/gy5Q8q, https://godbolt.org/z/7fm8Za 
and https://godbolt.org/z/9baieF, respectively.

The p10X0_YYY.cpp and p10X0e_YYY.cpp examples elaborate the p0000.cpp example.

The p11X0_YYY.cpp and p11X0e_YYY.cpp examples elaborate the p010X_YYY.cpp examples.

The p12X0e_YYY.cpp examples are a variant of the p11X0e_YYY.cpp examples,
with completes_synchronously() alternatively completing synchronously or asynchronously.

The p13X0_YYY.cpp and p13X0e_YYY.cpp examples use a number of coroutines 
that complete either on the same thread or on a dedicated thread (-thread in the name).

The p13X2_YYY.cpp and p13X2e_YYY.cpp examples are a variant in which coroutine2 "forgets" to co_await t; 
where t is the task returned by coroutine3.

The pXXX0_YYY.cpp examples use lazy start, the pXXX0e_YYY.cpp examples use eager start.

The pXX00_void.cpp and pXX00e_void.cpp examples use await_suspend() that returns void.

The pXX10_bool.cpp and pXX10e_bool.cpp examples use await_suspend() that returns bool.

The pXX20_coroutine_handle.cpp and pXX20e_oroutine_handle.cpp examples use await_suspend() that returns coroutine_handle<>.

The pXX30_corolib.cpp examples use async_ltask from corolib.

The pXX30e_corolib.cpp examples use async_task from corolib.
