# Study: Control flow

## Introduction

The reader is referred to [awaiter type variants](../../docs/awaiter_type_variants.md) for an introduction to awaiter types.

In this document we study the control flow in some simple functions and coroutines.
These examples serve as an introduction to a more detailed study of the behavior at the initial and final suspend points
in [initial_suspend](../initial_suspend) and  [final_suspend](../final_suspend), respectively.

Functions support two operations: call and return.
This is illustrated in the following scenario.

![legend01-function](./drawings/legend01-function.jpg)

Coroutines support four operations: call and return, and suspend and resume.
Therefore, coroutines can be considered to be a generalization of functions.
This is illustrated in the following scenario.

![legend02-coroutine](./drawings/legend02-coroutine.jpg)

Both diagrams are based upon UML sequence diagrams, with 2 differences.
* The vertical timelines correspond to functions and coroutines instead of objects.
* The horizontal arrows correspond to the 2 (4) operations on functions (coroutines) instead of member function calls on objects.

In more detail:
* Call and resume operations are depicted with full lines, return and suspend operation with dashed lines.
* A call/resume operation with number N must be followed with a return/suspend operation in the other direction with the same number.
* In case of functions, a call operation is always paired with a return operation with the same number.
* In case of coroutines, a call operation can be paired with a return operation with the same number.
* A call operation can also be paired with a suspend operation.
* A suspend operation requires a resume operation to continue the flow in the coroutine.
* Coroutines may not use the return operation, but suspend at their final awaiter point.
  In that case a 5th operation, destroy, must be used to release the coroutine state.

Note: the destroy operation will not be illustrated in the diagrams.

For a explanation of eager and lazy start, the reader is referred to [initial_suspend_](../initial_suspend]).

We consider 2 forms of completion:
* Synchronous completion: the leaf coroutine doesn't contain a co_await statement, only a co_return statement.
* Asynchronous completion: the leaf coroutine contains a co_await statement at which it will suspend and must be resumed later.

Synchronous completion is also possible if the leaf coroutine co_await's an awaitable that completes synchronously,
such as std::suspend_never. However, this does not change much to the basic explanation.

In case of asynchronous completion, we assume that this resumption takes place from main() and is
performed on the same thread as the original coroutine call.
In other words, the application uses only one thread.

This document analyzes the behavior of 4 coroutine examples:
* Synchronous completion with eager and with lazy start.
* Asynchronous completion with eager abd with lazy start.

A complete control flow encompasses both the initial and the final suspend point.

At the final suspend point, a user-defined type (final_awaiter) is used, with the following signature of await_suspend:

    void await_suspend(coroutine_handle<promise_type> h) noexcept

The reader is referred to [final_suspend_](../final_suspend]) for a more in-depth study of final awaiter types,
more in particular concerning avoiding memory leaks.

## Functions

The following scenario shows the control flow in an application that only uses functions.

Scenario:

![p0000 trace](./drawings/p0000.jpg)

There is no source code, because it is trivial to reconstruct the source code from the scenario.

## Coroutines: synchronous completion

Consider the following example 
(see [p2010e_void-sc.cpp](./p2010e_void-sc.cpp) 
 and [p2010l_void-sc.cpp](./p2010l_void-sc.cpp) for the full source code):

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

This code can be useful to follow the control flow sequences below, comparing eager and lazy start.

Note: In the (pseudo-)code above, return_to_caller_or_resumer() can be considered to be a kind of macro, 
that returns either a task object if the coroutine is called for the first time,
or a pointer to a coroutine state object if the coroutine is resumed from another coroutine or function.
A further transformation step will split foo() and bar() into two functions,
being a ramp function returning a task object and a resume function returning a pointer to a coroutine state object.
"return_to_caller_or_resumer();" can then be replaced with "return task;" or "return p_coroutine_state;" respectively.

For a more correct translation to pre C++20 code, the reader is referred to [transform](../transform).

### Eager start

Source code: [p2010e_void-sc.cpp](./p2010e_void-sc.cpp).

Scenario:

![p2010e_void-sc trace](./drawings/p2010e_void-sc.jpg)

The control flow sequence is as follows:

* main() calls bar() (1).
* bar() enters its initial suspend code section: bar() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_never.
    * bar() evaluates is.await_ready(), which returns true (2).
    * bar() calls is.await_resume() (3).
* bar() enters its user-authored code.
* bar() calls foo() (4).
* foo() enters its initial suspend code section: foo() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_never.
    * foo() evaluates is.await_ready(), which returns true (5).
    * foo() calls is.await_resume() (6).
* foo() enters its user-authored code.
* foo() executes co_return.
* foo() enters its final suspend code section: foo() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * foo() evaluates fs.await_ready(), which returns false (7).
    * foo() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately (8)
* foo() returns control to bar() at return_point_1.
* bar() saves the result value of foo() in task f.
* bar() co_awaits task f:
    * bar() evaluates f.await_ready(), which returns true because foo() ran to completion (9).
    * bar() calls f.await_resume() (10).
* bar() executes co_return.
* bar() enters its final suspend code section: bar() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * bar() evaluates fs.await_ready(), which returns false (11).
    * bar() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately (12).
* bar() returns control to main() at return_point_1.
* main() saves the return_value of bar() in b;
* main() calls b.start() (13). This function has an empty implementation in case of an eager task.
* main() calls b.get_result() (14). 
* main() returns.

Conclusion: The code does not suspend and consequently it does not have to be resumed.
The control flow is the same as if all coroutines were "ordinary" functions.

### Lazy start

Source code: [p2010l_void-sc.cpp](./p2010l_void-sc.cpp).

Scenario:

![p2010l_void-sc trace](./drawings/p2010l_void-sc.jpg).

The control flow sequence is as follows:

+ main() calls bar() (1).
* bar() enters its initial suspend code section: bar() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_always.
    * bar() evaluates is.await_ready(), which returns false (2).
    * bar() calls is.await_suspend() (3).
    * bar() returns control to its calling function at return_point_0.
* main() saves the return value of bar() in task b.
* main() calls b.start() (4).      
* start() resumes bar() (5). This is the first resume.
* We re-enter bar() at resume_point_0.
* bar() calls is.await_resume() (6).
* bar() enters its user-authored code section.
* bar() calls foo() (7).
* foo() enters its initial suspend code section: foo() calls co_await initial_suspend();
    * pr->initial_suspend() returns std::suspend_always.
    * foo() evaluates is.await_ready(), which returns false (8).
    * foo() calls is.await_suspend() (9).
    * foo() returns control to its calling function at return_point_0.
* bar() saves the return value of foo() in task f.
* bar() calls co_await f.
    * bar() evaluates f.await_ready(), which returns false (10).
    * bar() calls f.await_suspend() (11).
    * This implementation saves a coroutine_handle to bar() and it resumes foo() (12). This is the second resume.
* We re-enter foo() at resume_point_0.
* foo() calls is.await_resume() (13).
* foo() enters its user-authored code section.
* foo() executes co_return.
* foo() enters its final suspend code section: foo() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * foo() evaluates fs.await_ready(), which returns false (14).
    * foo() calls fs.await_suspend() (15).
    * The coroutine_handle to bar() is saved. foo() resumes bar() (16). This is the third resume.
* We re-enter bar() at resume_point_1.
* bar() calls f.await_resume() (17).
    * Notice that f.await_resume() is indirectly called from f.await_suspend(), which is indirectly called from b.start().
* bar() executes co_return;
* bar() enters its final suspend code section: bar() calls co_await final_suspend();
    * pr->final_suspend() returns final_awaiter.
    * bar() evaluates fs.await_ready(), which returns false (18).
    * bar() calls fs.await_suspend(). There is no coroutine to resume, so this function returns immediately (19).
* bar() returns control to its caller (resumer), which is fs.await_suspend() in foo() (16).
* In foo(), the call to fs.await_suspend() returns (15).
* foo() returns control to its caller, which is f.await_suspend() in bar() (12).
* In bar(), the call to f.await_suspend() returns (11).
* bar() returns control to its caller, which is b.start() (5).
* In main(), the call to b.start() returns (4).
* main() calls b.get_result() (20).
* main() returns.

Conclusion: The code suspends and consequently it has to be resumed 3 times.
This is a very complex control flow for something that is essentially a function call scenario
(main() calls a function bar() which calls a function foo()).

## Coroutines: asynchronous completion on the same thread

Consider the following example 
(see [p2020e_void-ma.cpp](./p2020e_void-ma.cpp) 
 and [p2020l_void-ma.cpp](./p2020l_void-ma.cpp) for the full source code):

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

### Eager start

Source code: [p2020e_void-ma.cpp](./p2012e_void-ma.cpp).

Scenario:

![p2020e_void-ma trace](./drawings/p2020e_void-ma.jpg).

### Lazy start

Source code: [p2020l_void-ma.cpp](./p2012l_void-ma.cpp).

Scenario:

![p2020l_void-ma trace](./drawings/p2020l_void-ma.jpg).
