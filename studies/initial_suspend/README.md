# Study initial_suspend(): lazy or eager start?

## Introduction

The reader is referred to [awaiter type variants](../../docs/awaiter_type_variants.md) for an introduction to awaiter types.

The examples in this folder are heavily inspired by the article
https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
and the corresponding code in https://godbolt.org/z/-Kw6Nf, https://godbolt.org/z/gy5Q8q, https://godbolt.org/z/7fm8Za 
and https://godbolt.org/z/9baieF

The main purpose of this study is to compare the advantages and disadvantages of
lazy start and eager start coroutines for the three variants of await_suspend(),
and the impact this choice has on the behavior of the application.

## Lazy versus eager start: overview

In a coroutine with return type 'task' and *lazy* start, task::promise_type::initial_suspend() returns std::suspend_always;
in a coroutine with return type 'task' and *eager* start, task::promise_type::initial_suspend() returns std::suspend_never.

The function await_suspend() can have one of the following three return types: void, bool, or coroutine_handle<>.

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
	        return false;
        await_suspend(std::coroutine_handle<promise_type> h) returns
	        void                    // see task_void.h and task_bool.h
	        bool                    // not used
	        std::coroutine_handle<> // see task_coroutine_handle.h
        await_resume() returns void

    task::promise_type::final_suspend() returns std::final_awaiter

For all three return types, task::awaiter::await_suspend() has to resume coro_.
In case of void and bool return types, this is accomplished by the statement coro_.resume();
in case of a std::coroutine_handle<> return type, await_suspend() has to return coro_.
The generated C++ code will then call coro_.resume().

Notice that any variant of task::awaiter::await_suspend() can be combined with any variant of
task::promise_type::final_awaiter::await_suspend(), leading to nine possible combinations.

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
	       void                         // see taske_void.h and taske_bool.h
	       bool                         // not used
	       std::coroutine_handle<>      // see taske_coroutine_handle.h
        await_resume() returns void

    task::promise_type::final_suspend() returns std::final_awaiter

For none of the three return types, task::awaiter::await_suspend() has to resume coro_.
In case of a std::coroutine_handle<> return type, await_suspend() has to return std::noop_coroutine().

Notice again that any variant of task::awaiter::await_suspend() can be combined with any variant of
task::promise_type::final_awaiter::await_suspend(), leading to nine possible combinations.

## Examples

### Example 1: Synchronous completion

Consider the following example 
(see [p2010e_void-sc.cpp](p2010e_void-sc.cpp) 
 and [p2010l_void-sc.cpp](p2010l_void-sc.cpp) for the full source code):

```c++
task foo() {
    co_return 1;
}

task bar() {
    task f = foo();
    int v = co_await f;
    co_return v+1;
}

int main() {
    task b = bar();
    b.start();
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    return 0;
}
```

The example uses synchronous completion:
function foo() doesn't contain a co_await statement, only a co_return statement.

After a few transform steps the (simplified) (pseudo-)code may look like:

```c++
task foo()
{
    // create coroutine state with embedded pr (task::promise_type) object
    // initial suspend point code section
    // co_await initial_suspend();
    auto is = pr->initial_suspend();
    // Lazy start coroutines:  is.await_ready() returns false
    // Eager start coroutines: is.await_ready() returns true
    if (!is.await_ready()) {
        save_resume_point(resume_point_0);
        is.await_suspend(...);
        return_to_caller_or_resumer();          // return_point_0
    }
resume_point_0:
    is.await_resume(...);
        
    try {
        // user-authored code section
        // co_return 1;
        return_value(1);
    }
    catch(...) {
        report_unhandled_exception();
    }

    // final suspend point code section
final_suspend_point:
    // co_await final_suspend();
    auto fs = pr->final_suspend();
    // returns false  (unless final_suspend() returns std::suspend_never)
    if (!fs.await_ready()) {
        mark_coroutine_as_done();
        fs.await_suspend(...);
        return_to_caller_or_resumer();          // return_point_1
    }
    fs.await_resume();                          // This code may never be reached
    return_to_caller_or_resumer();              // return_point_2
} // foo

task bar()
{
    // create coroutine state with embedded pr (task::promise_type) object
    // initial suspend point code section
    // co_await initial_suspend();
    auto is = pr->initial_suspend();
    // Lazy start coroutines: is.await_ready() returns false
    // Eager start coroutines: is.await_ready() returns true
    if (!is.await_ready()) {
        save_resume_point(resume_point_0);
        is.await_suspend(...);
        return_to_caller_or_resumer();          // return_point_0
    }
resume_point_0:
    is.await_resume(...);

    try {
        // user-authored code section
        task f = foo();
        // int v = co_await f;
        if (!f.await_ready()) {
            // coroutine is considered to be suspended at this point
            save_resume_point(resume_point_1);
            f.await_suspend(...);
            return_to_caller_or_resumer();       // return_point_1
        }
resume_point_1:
        // coroutine is considered to be resumed here
        int v = f.await_resume();

        // co_return v+1;
        return_value(v+1);
    }
    catch(...) {
        report_unhandled_exception();
    }

    // final suspend point code section
final_suspend_point:
    // co_await final_suspend();
    auto fs = pr->final_suspend();
    // returns false (unless final_suspend() returns std::suspend_never)
    if (!fs.await_ready()) {
        mark_coroutine_as_done();
        fs.await_suspend(...);
        return_to_caller_or_resumer();          // return_point_2
    }
    fs.await_resume();                          // This code may never be reached
    return_to_caller_or_resumer();              // return_point_3
} // bar

int main() {
    task b = bar();
    b.start();
    return 0;
}
```

This code can be useful to follow the explanations below, comparing eager and lazy start.

Note: In the (pseudo-)code above, return_to_caller_or_resumer() can be considered to be a kind of macro, 
that returns either a task object if the coroutine is called for the first time,
or a pointer to a coroutine state object if the coroutine is resumed from another coroutine or function.
A further transformation step will split foo() and bar() into two functions,
being a ramp function returning a task oobject and a resume function returning a pointer to a coroutine state object.
"return_to_caller_or_resumer();" can then be replaced with "return task;" or "return p_coroutine_state;" respectively.

#### Eager start

The control flow sequence is as follows:

1. main() calls bar().
2. bar() enters its initial suspend code section: bar() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_never.
    * bar() evaluates is.await_ready(), which returns true.
    * bar() calls is.await_resume().
3. bar() enters its user-authored code.
4. bar() calls foo().
5. foo() enters its initial suspend code section: foo() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_never.
    * foo() evaluates is.await_ready(), which returns true.
    * foo() calls is.await_resume().
6. foo() enters its user-authored code.
7. foo() executes co_return.
8. foo() enters its final suspend code section: foo() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * foo() evaluates fs.await_ready(), which returns false.
    * foo() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately.
9. foo() returns control to bar() at return_point_1.
10. bar() saves the result value of foo() in task f.
11. bar() co_awaits task f:
    * bar() evaluates f.await_ready(), which returns true because foo() ran to completion.
    * bar() calls f.await_resume().
12. bar() executes co_return.
13. bar() enters its final suspend code section: bar() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * bar() evaluates fs.await_ready(), which returns false.
    * bar() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately.
14. bar() returns control to main() at return_point_1.
15. main() saves the return_value of bar() in b;
16. main() calls b.start(). This function has an empty implementation in case of an eager task.
17. main() returns.

Conclusion: The code does not suspend and consequently it does not have to be resumed.
The control flow is the same as if all coroutines were "ordinary" functions.

#### Lazy start

The control flow sequence is as follows:

1. main() calls bar().
2. bar() enters its initial suspend code section: bar() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_always.
    * bar() evaluates is.await_ready(), which returns false.
    * bar() calls is.await_suspend().
    * bar() returns control to its calling function at return_point_0.
3. main() saves the return value of bar() in task b.
4. main() calls b.start().      
5. start() resumes bar(). This is the first resume.
6. We re-enter bar() at resume_point_0.
7. bar() calls is.await_resume().
8. bar() enters its user-authored code section.
9. bar() calls foo().
10. foo() enters its initial suspend code section: foo() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_always.
    * foo() evaluates is.await_ready(), which returns false.
    * foo() calls is.await_suspend().
    * foo() returns control to its calling function at return_point_0.
11. bar() saves the return value of foo() in task f.
13. bar() calls co_await f.
    * bar() evaluates f.await_ready(), which returns false.
    * bar() calls f.await_suspend(). This implementation saves a coroutine_handle to bar() and it resumes foo(). This is the second resume.
14. We re-enter foo() at resume_point_0.
15. foo() calls is.await_resume().
16. foo() enters its user-authored code section.
17. foo() executes co_return.
18. foo() enters its final suspend code section: foo() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * foo() evaluates fs.await_ready(), which returns false.
    * foo() calls fs.await_suspend().
    * The coroutine_handle to bar() is saved. foo() resumes bar(). This is the third resume.
18. We re-enter bar() at resume_point_1.
19. bar() calls f.await_resume().
    * Notice that f.await_resume() is indirectly called from f.await_suspend(), which is indirectly called from b.start().
20. bar() executes co_return;
21. bar() enters its final suspend code section: bar() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * bar() evaluates fs.await_ready(), which returns false.
    * bar() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately.
22. bar() returns control to its caller, which is foo().
23. In foo(), the call to fs.await_suspend() returns.
24. foo() returns control to its caller, which is the f.await_suspend() call in bar().
25. In bar(), the call to f.await_suspend() returns.
26. bar() returns control to its caller, which is b.start().
27. In main(), the call to b.start() retuns.
28. main() returns.

Conclusion: The codes suspends and consequently it has to be resumed 3 times.
This is a very complex control flow for something that is essentially a function call scenario
(main() calls a function bar() which calls a function foo()).

### Example 2: Asynchronous completion on the same thread

Consider the following example 
(see [p2020e_void-ma.cpp](p2020e_void-ma.cpp) 
 and [p2020l_void-ma.cpp](p2020l_void-ma.cpp) for the full source code):

```c++
mini_awaiter ma;

task foo() {
    int v = co_await ma;
    co_return v+1;
}

task bar() {
    task f = foo();
    int v = co_await f;
    co_return v+1;
}

int main() {
    task b = bar();
    b.start();
    ma.set_result_and_resume(10);
    int v = b.get_result();
    // Use v
    return 0;
}
```

#### Eager start

Scenario: TBC

#### Lazy start

Scenario: TBC

### Example 3: Asynchronous completion on another thread

Consider the following example 
(see [p2030e_void-ma-thread.cpp](p2020e_void-ma-thread.cpp) 
 and [p2030l_void-ma-thread.cpp](p2030l_void-ma-thread.cpp) for the full source code):

```c++
task foo() {
    int v = co_await ma;
    co_return v+1;
}

task bar() {
    task f = foo();
    int v = co_await f;
    co_return v+1;
}

int main() {
    task b = bar();
    b.start();

    std::thread thread1([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ma.set_result_and_resume(10);
        });
    thread1.join();

    int v = b.get_result();
    // Use v
    return 0;
```

#### Eager start

Scenario: TBC

#### Lazy start

Scenario: TBC

## Lazy versus eager start: evaluation

### Lazy start: await_ready() obsolete?

In case of lazy start, all await_ready() implementations 
(in std::suspend_always, task::awaiter and task::promise_type::final_awaiter) return hard-coded false.
This means that await_suspend() is called unconditionally.

Therefore, the await_ready() function can be considered to be obsolete:
the coroutine will always suspend at a co_await statement and so,
it will have to be resumed to enter the user-authored body,
often from a co_await statement in another coroutine.
(In case of the initial suspend point, this will usually be the calling coroutine.)

This could be an indication that it never was the intention to use the co_await implementation for lazy-start coroutines.
See also the next point.

### Lazy start: semantic issues

Lazy start coroutines seem to alter the intended/original/intuitive meaning of "co_await coroutine1(...)".
co_await should suspend a coroutine if the called coroutine (coroutine1 in this case)
cannot provide the reply to its caller right away.

In case of lazy start coroutines, co_await seems to mean "to start the called coroutine" instead,
thus altering its intuitive meaning (semantics).
A better name for co_await in this case is co_start. 

When you await the arrival a person, this usually doesn't mean that you have to put that person on the road to you.

Python and C# async/await use the more intuitive eager start variant.

### Lazy start: performance loss

Lazy start introduces suspend-resume operations on the calling path.

When coroutine1 calls coroutine2, which calls coroutine3, which calls coroutine4, etc.,
then suspend-resume operations will also be used on the calling path,
and not only on the return path (in case a coroutine cannot provide the reply to its caller).

There will be many more suspend-resume operations than in case of eager start coroutines.
This leads to performance loss, all for code that is invisible to the application writer.

### Lazy start coroutines follow a very complex path in case of synchronous completion

This is illustrated by example 1.

Eager start coroutines follow the same control flow
as normal function calls, because that is what synchronous completion means.
There are additional steps, but these steps do not change the control flow.

The lazy start coroutines 

### Lazy start may lead to mutually recursive calls when a coroutine is called in a loop

The following is the core example in https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer.

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
the coroutines loop_synchronously(int count) and completes_synchronously() call each other recursively.

With the împlementation of the task class in https://godbolt.org/z/-Kw6Nf, 
which is repeated here in p1000_void.cpp and in task_void.h,
the application crashes for "large" values of count.

The reason for this mutual recursion is the use of a lazy start coroutine type and not so much the fact that 
task::promise_type::final_awaiter::await_suspend(coroutine_handle<promise_type>) and
task::awaiter::await_suspend(coroutine_handle<>) return void.

Indeed, by using an eager start coroutine type instead,
as is done in taske_void.h, both coroutines do not call each other recursively anymore.
Instead, the control flow is the same as that of normal functions: the coroutines do not suspend and resume,
but just use call and return.

The solution to this problem required the introduction of symmetric transfer.
Symmetric transfer does not eliminate mutual recursive calls, but it avoids that the stack keeps growing
by relying on the use of tail recursion.

### Eager start coroutines can be started automatically from main()

Eager start coroutines can be called and started from normal functions, in particular main().
The reason is that you don't have to co_await such a coroutine to resume them beyond the initial suspend point.

The code that comes with https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer
introduces an eager start coroutine type, sync_wait_task:
sync_wait_task::promise_type::initial_suspend() returns std::suspend_never.
The static sync_wait_task start(task&& t) coroutine co_awaits the task object passed to it.

To start a lazy start coroutine, it suffices to add a start() function to the task class.
This function will resume the coroutine via its coroutine_handle.

I have added an implementation of this function to the task classes in task_void.h, task_bool.h and task_coroutine.h:

```c++
    void start() {
        if (coro_)
            coro_.resume();
    }
```

In corolib, the start function has been implemented in async_ltask (where ltask stands for lazy task).
A dummy (empty) implementation has been added in async_task.

### Completion on another thread

#### Source code

Consider the following code:

```c++
task foo() {
    operation1 op;
    int v = co_await op;
    // post co_await code
    co_return v+1;
}

task bar() {
    task f = foo();
    int v = co_await f;
    // post co_await code
    co_return v+1;
}

int main() {
    task b = bar();
    b.start();
    // waiting point
    enter_event_loop();
    // Use v
    return 0;
}
```

The coroutine return type 'task' can use eager or lazy start, and so can 'operation1'.
Note that operation1 is an awaitable, not a coroutine return type: operation1 does not define a promise_type.

Four combinations are possible.

* eager start task + eager start operation1, see [p2050e_void-op1e-thread.cpp](p2050e_void-op1e-thread.cpp) and [p2055e_void-op1e-thread.cpp](p2055e_void-op1e-thread.cpp)

* lazy start task + eager start operation1, see [p2050l_void-op1e-thread.cpp](p2050l_void-op1e-thread.cpp) and [p2055l_void-op1e-thread.cpp](p2055l_void-op1e-thread.cpp)

* eager start task + lazy start operation1, see [p2060e_void-op1l-thread.cpp](p2060e_void-op1l-thread.cpp) and [p2065e_void-op1l-thread.cpp](p2065e_void-op1l-thread.cpp)

* lazy start task + lazy start operation1, see [p2060l_void-op1l-thread.cpp](p2060l_void-op1l-thread.cpp) and [p2065l_void-op1l-thread.cpp](p2065l_void-op1l-thread.cpp)

#### Evaluation

When using a lazy start task, the await_suspend() function starts the coroutine the task refers-to.
Likewise, when using a lazy start operation, the await_suspend() function starts the real operation
(which will run to completion on the completion thread).
To enter await_suspend(), await_ready() function has to return false (hard-coded).
When the call to await_suspend() returns, the coroutine returns control to its caller (or resumer).
In short, a lazy start task or operation starts the "real work" just before it returns.
There should be no need to access any variables after the task or operation has started the work and before it returns.
If so, there is a possibility that these variables are also accessed from the completion thread.
This process is repeated for the mentioned caller or resumer, etc.

At some point, however, we will reach a point where we cannot "descend" (i.e. suspend) any further,
otherwise we would "fall off" the original thread; let's call this point the "waiting point."

At some time, the operation will complete on another thread.
This is even possible while the original thread is still descending towards the waiting point.

To make things a bit easier, let's assume that the original thread reaches its waiting point before the operation completes.

At the waiting point, we have the following options:
* the application waits until the operation has completed to process the result
    * and then exit the application or
    * start waiting for new input.
* the application can start waiting for new input and start processing this input even if the first operation has not yet completed.

Alternative 1: the application has to wait for the completion at the waiting point.
The original thread will have to be informed by the completion thread.
The "standard" way to do this, is to use a semaphore that is acquired by the original thread and released by the completion thread.

Alternative 2: the application can start processing new input.
This is the most valuable case. If this case was not allowed, then we are in essence writing synchronous applications...
When the firt operation completes, the completion thread may run concurrently
with the original thread that is already processing a new event:
the "post co_await code" in foo() and bar() will run on the completion thread.
If this code manipulates any data that is also accessed while processing the new event, data has to
protected against concurrent access.
 
In other words: although the use of lazy start coroutines 
avoids the need for thread synchronization in infrastructure classes such as task or operation1,
thread synchronization issues cannot be avoided when the completion thread runs concurrently
with the original thread that is processing new input?

#### Solution

The best solution is to have the completion thread run no application code at all,
but instead create and post an event to an event queue.
This event will be handled in the event loop as any other new event.
The event queue has to protected against concurrent access, of course;
however, all application code will run on the same thread.
The event queue can be part of the coroutine library or the communnication framework.

corolib provides a class, ThreadAwaker, that allows the original thread to signal
to one or more completion threads that they may proceed.
The original thread will do this just before it reaches the waiting point.
This solution should be combined with the one described in the previous paragraph.

For corolib examples, the user is referred to the tutorial, more in particular to the use of UseMode::USE_THREAD_QUEUE
in the pXXXX-async_operation-thread-queue.cpp examples.

### task object of foo() goes out of scope

```c++
task bar()
{
    task f = foo();
    // point 1
    // Pre co_await code
    // co_await f;         
    // Post co_await code
    co_return;
}
```

The co_await f; statement has been commented out.
Likewise, if present it will not be reached, e.g. because it is placed in an if-block whose condition evaluates to false.

At the end of bar(), task object f goes out of scope and it will (try to) destroy foo()'s coroutine frame.

Let's compare lazy and eager start.

* Lazy start: foo() has suspended at its initial suspend point. The task destructor can safely destroy foo()'s coroutine frame
  because foo() has not entered its body (and started an operation that has to be completed).

* Eager start(): foo() may (will) have started an operation that may not have completed.
  The task destructor must check if foo() is done().
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

### Exception in pre co_await code

```c++
task bar()
{
    task f = foo();
    // Point 1
    // pre co_await code can raise an exception
    co_await f;         
    // post co_await code
    co_return;
}
```

Let's compare lazy and eager start.

* Lazy start: foo() has suspended at its initial suspend point. Even if the exception is not caught,
  the task destructor can safely destroy foo()'s coroutine frame
  because foo() has not entered its body (and started an operation that has to be completed).

* Eager start: foo() may (will) have started an operation that may not have completed. The co_await statement will be skipped.
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

The p1XX0_YYY.cpp examples use lazy start, the p1XX0e_YYY.cpp examples use eager start.

The p1XX0l_YYY.cpp examples use lazy start, but without using manual_executor and sync_wait_task as in the p1XX0_YYY.cpp examples.
The p1XX0l_YYY.cpp and pXXX0e_YYY.cpp are very close in style.

The p1X00_void.cpp, p1X00l_void.cpp and p1X00e_void.cpp examples use await_suspend() that returns void.

The p1X10_bool.cpp, p1X10l_bool.cpp and p1X10e_bool.cpp examples use await_suspend() that returns bool.

The p1X20_coroutine_handle.cpp, p1X20l_coroutine_handle.cpp and p1X20e_oroutine_handle.cpp examples 
use await_suspend() that returns coroutine_handle<>.

The p13X0_corolib.cpp examples use async_ltask from corolib.

The p130Xe_corolib.cpp examples use async_task from corolib.
