# Avoiding memory leaks

## Introduction

The reader is referred to [awaiter type variants](../../docs/awaiter_type_variants.md) for an introduction to awaiter types.

The original implementation of [async_task](../../include/corolib/async_task.h)
using std::suspend_always at the final suspend point had memory leaks
because not all coroutine frames (with their "embedded" promise_type objects) were always released.

This document explains the causes of the problem and how it can be solved.

## Example

The explanation below uses code from [class_async.h](class_async.h)

```c++
extern mini_awaiter are1;

class Class
{
public:
    task coroutine4() {
        co_await are1;                  // coroutine4 will suspend here and pass control to coroutine3()
                                        // coroutine4 will be resumed from main() and run to completion
        co_return 1;                    // coroutine4 will resume coroutine3 and return to the are1.resume() call in main().
    }

    task coroutine3() {
        int v = co_await coroutine4();  // coroutine3 will suspend here and pass control to coroutine2()
                                        // coroutine3 will be resumed from coroutine4() and run to completion
        co_return v+1;                  // coroutine3 will resume coroutine2 and then return to coroutine4
    }

    task coroutine2() {
        int v = co_await coroutine3();  // coroutine2 will suspend here and pass control to coroutine1()
                                        // coroutine2 will be resumed from coroutine3() and run to completion
        co_return v+1;                  // coroutine2 will resume coroutine1 and then return to coroutine3
    }

    task coroutine1() {
        int v = co_await coroutine2();  // coroutine1 will suspend here and pass control to main()
                                        // coroutine1 will be resumed from coroutine2() and run to completion
        co_return v+1;                  // There is no coroutine to resume, just save the result and
                                        // return to coroutine2
    }
};
```

and with the main() function from (e.g.) [p1010e_sa.cpp](p1010e_sa.cpp)

```c++
    int main() {
        Class obj;
        task a = obj.coroutine1();
        are1.resume();                  // This call will resume coroutine4()
                                        // "are" stood originally for "auto reset event"
        int v = a.get_result();
        return 0;
    }
```

mini_awaiter is defined in [mini_awaiter.h](mini_awaiter.h):

```c++
struct mini_awaiter
{
    std::coroutine_handle<> m_awaiting;

    void resume()                   // This is a resume function defined at the coroutine library level.
                                    // It is possible to use another name to avoid any confusion 
                                    // with the resume() function below.
    {
        if (!m_awaiting.done())
            m_awaiting.resume();    // This is std::coroutine_handle<>::resume() defined in the coroutine header file.
    }

    auto operator co_await() noexcept
    {
        // Implementation not included here to save some space
    }
};
```

## Overview

The rest of this document will discuss the following alternatives:

* final_suspend() returns std::suspend_always (std::suspend_always::await_ready() returns false)
* final_suspend() returns std::suspend_never (std::suspend_never::await_ready() returns true)
* final_suspend() returns a custom 'final_awaiter' type where:
    * final_awaiter::await_suspend() returns void, or
    * final_awaiter::await_suspend() returns bool, or
    * final_awaiter::await_suspend() returns coroutine_handle<>

Note: the statement 'co_return ret;' where ret has type TYPE, will be transformed into a call of return_value(TYPE),
whereas the statement 'co_return;' will be transformed into a call of return_void().

In the first two cases (using std::suspend_always or std::suspend_never at the final suspend point),
the resumption of the previous coroutine (coroutineN+1 resumes coroutineN)
will be implemented in the functions return_value(TYPE) or return_void().

When using a custom 'final_awaiter' type, we can distinguish between two cases:

* If final_awaiter::await_ready() returns false
    * the coroutine will suspend
    * final_awaiter::await_suspend() is called (3 variants, see above)
        * resumption of the calling coroutine will be implemented in this function
    * final_awaiter::await_resume() is not called
    * (This can be considered to be an extension of std::suspend_always.)
* If final_awaiter::await_ready() returns true
    * the coroutine will not suspend; it will run till the end and then delete its state object
    * final_awaiter::await_suspend() is not called (so the 3 variants do not matter, the bool variant will be used below)
    * final_awaiter::await_resume() is called
        * resumption of the calling coroutine will be implemented in this function
    * (This can be considered to be an extension of std::suspend_never.)

We will see that only the use of a final_awaiter type can avoid memory leaks.

## final_suspend() returns std::suspend_always

This corresponds to the original approach used in the implementation
of class [async_task](../../include/corolib/async_task.h)

File [taske_sa.h](taske_sa.h) contains an implementation of a 'task' class that uses eager start (this explains the 'e' in the file name).
File [task_sa.h](task_sa.h) contains another implementation of a 'task' class that uses lazy start.

'sa' is short for the use of std::suspend_always at the final suspend point.

struct suspend_always can be defined as follows:

```c++
struct suspend_always {
     constexpr suspend_always() noexcept = default;
     constexpr bool await_ready() const noexcept { return false; }
     constexpr void await_suspend(coroutine_handle<>) const noexcept {}
     constexpr void await_resume() const noexcept {}
};
```

Coroutines returning an eager-start task object can be started from main():
there is no need to apply co_await to the task object returned from a coroutine ("coroutine return object") 
to move it beyond its initial suspend point (see the next code fragment),
nor do we need a special start function for this purpose.

A first high-level transformation of coroutine1()

```c++
    task coroutine1() {
        int v = co_await coroutine2();
        co_return v+1;
    }
```

may look as follows:

```c++
    task coroutine1() {
        coroutine1_state* __state = new coroutine1_state{};
        save_frame_pointer(__state);
        task __return = __state->_promise.get_return_object();
        
        // Initial suspend point
        co_await __state->_promise.initial_suspend();

        try {
            // Original code
            int v = co_await coroutine2();
            co_return v+1;
        }
        catch (...) {
            __state->_promise.unhandled_exception();
        }

        // Final suspend point
        co_await __state->_promise.final_suspend();
    }
```

The next transformation step will most probably split coroutine1() into a "ramp function," 
a "resume function" and a "destroy" function.
See https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform for more information.

The ramp function has the same signature as the original coroutine:

```c++
    task coroutine1()
```

The resume function has the following signature:

```c++
    __coroutine_state* __coroutine1_resume(__coroutine_state*)
```

Finally, the destroy function has the following signature:

```c++
    void __coroutine1_destroy(__coroutine_state*)
```

Function task::promise_type::return_value() is implemented as follows:

```c++
    void return_value(int v)
    {
        value = v;
        if (continuation)
            continuation.resume();    // We resume the co_awaiting coroutine from here
    }
```

Function task::promise_type::final_suspend() returns std::suspend_always:

```c++
    auto final_suspend() noexcept {
        return std::suspend_always{};
    }
```

The last part of the __coroutine1_resume() function (from the co_return statement till the end)
may look as follows:

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* __coroutine1_resume(__coroutine_state* s) {
        auto* state = static_cast<coroutine1_state*>(s);
        ...
        ...
            // co_return v + 1;
            state->_promise.return_value(v + 1);
            goto final_suspend_point;
        }
        catch (...) {
            state->_promise.unhandled_exception();
        }
    final_suspend_point:
        // co_await state->_promise.final_suspend();
        if (!state->_fa.await_ready()) {    // await_ready() always returns false in case of suspend_always
                                            // So we enter this code block: the coroutine is now suspended.
            state->_suspend_point = N;      // N depends on the previous number of suspend points
            state->_resume = nullptr;       // Mark as final suspend point: done() will return true
            state->_fa.await_suspend(
                std::coroutine_handle<__coroutine1_promise_t>::from_promise(state->_promise));
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
                                            // We left __coroutine1_resume() at this point
        }
                                            // We will never reach this point!
        state->__fa.await_resume();    
        delete state;                       // Consequently, we will not delete state (here).
        return static_cast<__coroutine_state*>(std::noop_coroutine().address());
    }
```

Using suspend_always, the coroutine unconditionally suspends at the final suspend point:
the coroutine state and the promise_type object inside the coroutine state are not (yet) destroyed.
This way, the calling coroutine (or function) can still access the result of the called coroutine 
via the coroutine return object, e.g. by calling function task::get_result():

```c++
    int get_result() {
        if (coro_)
            return coro_.promise().value;
        else
            return -1;
    }
```

Such a "result access function" is necessary because the main() function cannot use co_await to get access to the result.
Using co_await in main() would turn main() into a coroutine: what type should main() return?

However, there is no resume() call anywhere in the transformed code
that will eventually force the coroutine to proceed to the end:
this will lead to memory leaks.

To (try to) avoid this, the destructor of the coroutine return object (task) must call the destroy() function as follows:

```c++
    ~task() {
        print(PRI2, "%p: task::~task()\n", this);
        if (coro_)                  // Does the coroutine_handle still contain a valid pointer to the coroutine state?
            if (coro_.done()) {     // Has the coroutine reached the final suspend point (and reset the "resume" function pointer)?
                coro_.destroy();    // Call "destroy" function
                coro_ = {};
            }
            else {
                print(PRI2, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
```

With the implementation of return_value(int val) above,
coroutineN+1 resumes its calling coroutine coroutineN in its co_return statement
(that translates to return_value(...)).

However, in the resume flow of the example application (see first section),
the coroutine return object that is returned by coroutineN+1 and co_await-ed upon by coroutineN,
will go out of scope *before* coroutineN+1 has reached its final suspend point.
This is true for all coroutineN.

For example, in the transform of coroutine2(), we find the following code:

```c++
        try {
            // Original code
            int v = co_await coroutine3(); 	// Intermediate task object will go out of scope
            co_return v+1;               	// coroutine2 will resume coroutine1,
                                            // but was itself resumed from the co_return statement in coroutine3.
                                            // coroutine2 will then return to coroutine3.
        }
        catch (...) {
            __state->_promise.unhandled_exception();
        }
    final_suspend_point:
         // see above
```

or, when slightly rewritten to show the temporary task object:

```c++
        try {
            // Original code
            task t = coroutine3();
            int v = co_await t;
            co_return v + 1;                    // resume coroutine1 
                                                // t goes out of scope here
        }
        catch (...) {
            __state->_promise.unhandled_exception();
        }
    final_suspend_point:
	     // see above
```

In other words, the destructor of the task object
will be called *before* coroutine coroutineN+1 has reached its final suspend point.
Consequently, coro_.done() returns false and ~task will not call coro_.destroy().

This following is a trace of p1010e_sa.cpp (compiled into stfs-p1010e_sa.exe on Windows with set_print_level(0x03);).

```
PS ...> ./stfs-p1010e_sa.exe
00: main(): task a = obj.coroutine1();
00: 0000017E13F8FB60: promise_type::promise_type()
00: 0000004AA38FF708: task::task(...)
00: coroutine1(): int v = co_await coroutine2();
00: 0000017E13F93CF0: promise_type::promise_type()
00: 0000017E13F8FB90: task::task(...)
00: coroutine2(): int v = co_await coroutine3();
00: 0000017E13F93E10: promise_type::promise_type()
00: 0000017E13F93D20: task::task(...)
00: coroutine3(): int v = co_await coroutine4();
00: 0000017E13F93F30: promise_type::promise_type()
00: 0000017E13F93E40: task::task(...)
00: coroutine4(): co_await are1;
00: main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));
00: main(): are1.resume();
00: coroutine4(): co_return 1;
00: 0000017E13F93E40: task::~task(): test on coro_.done()
00: 0000017E13F93E40: task::~task(): !coro.done()
00: coroutine3(): co_return 2;
00: 0000017E13F93D20: task::~task(): test on coro_.done()
00: 0000017E13F93D20: task::~task(): !coro.done()
00: coroutine2(): co_return 3;
00: 0000017E13F8FB90: task::~task(): test on coro_.done()
00: 0000017E13F8FB90: task::~task(): !coro.done()
00: coroutine1(): co_return 4;
00: main(): int v = a.get_result();
00: main(): v = 4;
00: main(): return 0;
00: 0000004AA38FF708: task::~task(): test on coro_.done()
00: 0000004AA38FF708: task::~task(): coro_.destroy();
00: 0000017E13F8FB60: promise_type::~promise_type()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       1       3       4
00: fin 4       1       3       4
00: --------------------------------------------------------
00: Waiting 1000 milliseconds before exiting
PS ...>
```

The constructor (cons column) of the cor (coroutine return type = task object) and pro (promise_type) objects
is called 4 times. The destructor (dest column) is called 4 times for the task object, but only once
for the promise_type object.
This means that 3 coroutine states with their "embedded" promise_type objects are leaked.
These 3 leaks correspond to the 3 lines containing "task::~task(): !coro.done()."
Finally, there are maximum (max column) 4 task objects and promise_type objects alive at the same time.

What if we rewrote the task destructor as follows?

```c++
    ~task() {
        print(PRI2, "%p: task::~task()\n", this);
        if (coro_) {            // Does the coroutine_handle still contain a valid pointer to the coroutine state?
            coro_.destroy();    // Call "destroy" function
            coro_ = {};
        }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
```

No more memory leaks, but two task destructors are called twice for the same object, see trace below:

```
PS ...> ./stfs2-p1010e_sa.exe
00: main(): task a = obj.coroutine1();
00: 0000015C72779F30: promise_type::promise_type()
00: 00000054FE6FFA18: task::task(...)
00: coroutine1(): int v = co_await coroutine2();
00: 0000015C7277A050: promise_type::promise_type()
00: 0000015C72779F60: task::task(...)
00: coroutine2(): int v = co_await coroutine3();
00: 0000015C7277A170: promise_type::promise_type()
00: 0000015C7277A080: task::task(...)
00: coroutine3(): int v = co_await coroutine4();
00: 0000015C7277A290: promise_type::promise_type()
00: 0000015C7277A1A0: task::task(...)
00: coroutine4(): co_await are1;
00: main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));
00: main(): are1.resume();
00: coroutine4(): co_return 1;
00: 0000015C7277A1A0: task::~task(): no test on coro_.done()
00: 0000015C7277A1A0: task::~task(): coro_.destroy();
00: 0000015C7277A290: promise_type::~promise_type()
00: coroutine3(): co_return 2;
00: 0000015C7277A080: task::~task(): no test on coro_.done()
00: 0000015C7277A080: task::~task(): coro_.destroy();
00: 0000015C7277A1A0: task::~task(): no test on coro_.done()
00: 0000015C7277A1A0: task::~task(): coro_ == nullptr
00: 0000015C7277A170: promise_type::~promise_type()
00: coroutine2(): co_return 3;
00: 0000015C72779F60: task::~task(): no test on coro_.done()
00: 0000015C72779F60: task::~task(): coro_.destroy();
00: 0000015C7277A080: task::~task(): no test on coro_.done()
00: 0000015C7277A080: task::~task(): coro_ == nullptr
00: 0000015C7277A050: promise_type::~promise_type()
00: coroutine1(): co_return 4;
00: main(): int v = a.get_result();
00: main(): v = 4;
00: main(): return 0;
00: 00000054FE6FFA18: task::~task(): no test on coro_.done()
00: 00000054FE6FFA18: task::~task(): coro_.destroy();
00: 0000015C72779F30: promise_type::~promise_type()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       6       -2      4
00: pro 4       4       0       4
00: fin 4       1       3       4
00: --------------------------------------------------------
00: Waiting 1000 milliseconds before exiting
PS ...>
```

In this simple program there is no fatal error (we enter the destructor but leave it immediately because coro_ == nullptr), 
but in general, calling destroy() on a coroutine_handle of a coroutine that has
not reached its final suspend point yet can lead to serious misbehavior (such as segmentation faults)
if the coroutine is resumed after its coroutine state has been destroyed.

## final_suspend() returns std::suspend_never

Instead of final_suspend() returning std::suspend_always, what would be the result if final_suspend() returns
std::suspend_never?

struct suspend_never can be defined as follows:

```c++
struct suspend_never {
     constexpr suspend_never() noexcept = default;
     constexpr bool await_ready() const noexcept { return true; }           // only difference with suspend_always
     constexpr void await_suspend(coroutine_handle<>) const noexcept {}
     constexpr void await_resume() const noexcept {}
};
```

### First implementation

File [taske_sn.h](taske_sn.h) contains an implementation of a 'task' class that uses eager start (this explains the 'e' in the file name).
File [task_sn.h](task_sn.h) contains another implementation of a 'task' class that uses lazy start.

'sn' is short for the use of std::suspend_never at the final suspend point.

Looking at the generated code, we see that the code will now run till the end of the __coroutine1_resume() function.

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* __coroutine1_resume(__coroutine_state* s) {
        auto* state = static_cast<coroutine1_state*>(s);
        ...
        ...
            // co_return v + 1;
            state->_promise.return_value(v + 1);
            goto final_suspend_point;
        }
        catch (...) {
            state->_promise.unhandled_exception();
        }
    final_suspend_point:
        // co_await state->_promise.final_suspend();
        if (!state->_fa.await_ready()) {    // await_ready() always returns true in case of suspend_never
                                            // So we skip this code block
            state->_suspend_point = N;      // N depends on the previous number of suspend points
            state->_resume = nullptr;       // mark as final suspend point: done() will return true
            state->_fa.await_suspend(
                std::coroutine_handle<__coroutine1_promise_t>::from_promise(state->_promise));
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
                                            // We will not return here
        }
                                            // We will always execute this code
        state->__fa.await_resume();         
        delete state;                       // Consequently, we will always delete state
        return static_cast<__coroutine_state*>(std::noop_coroutine().address());
                                            // And return "std::noop_coroutine()"
    }
```

If is obvious that the coroutine frame will always be deleted: there is no fear anymore for memory leaks.
Unfortunately, it is deleted too soon, i.e. before main() can retrieve the result from the promise_type object
in the deleted frame.

The following is the last part of the trace of running .\stfs-p1110e_sn.exe (compiled with set_print_level(0x03);):

```
PS ...> .\stfs-p1110e_sn.exe
00: main(): task a = obj.coroutine1();
00: 0000024FDE296C50: promise_type::promise_type()
00: 000000F411AFF698: task::task(...)
00: coroutine1(): int v = co_await coroutine2();
00: 0000024FDE296D70: promise_type::promise_type()
00: 0000024FDE296C80: task::task(...)
00: coroutine2(): int v = co_await coroutine3();
00: 0000024FDE296E90: promise_type::promise_type()
00: 0000024FDE296DA0: task::task(...)
00: coroutine3(): int v = co_await coroutine4();
00: 0000024FDE296FB0: promise_type::promise_type()
00: 0000024FDE296EC0: task::task(...)
00: coroutine4(): co_await are1;
00: main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));
00: main(): are1.resume();
00: coroutine4(): co_return 1;
00: 0000024FDE296EC0: task::~task(): test on coro_.done()
00: 0000024FDE296EC0: task::~task(): !coro.done()
00: coroutine3(): co_return 2;
00: 0000024FDE296DA0: task::~task(): test on coro_.done()
00: 0000024FDE296DA0: task::~task(): !coro.done()
00: coroutine2(): co_return 3;
00: 0000024FDE296C80: task::~task(): test on coro_.done()
00: 0000024FDE296C80: task::~task(): !coro.done()
00: coroutine1(): co_return 4;
00: 0000024FDE296C50: promise_type::~promise_type()
00: 0000024FDE296D70: promise_type::~promise_type()
00: 0000024FDE296E90: promise_type::~promise_type()
00: 0000024FDE296FB0: promise_type::~promise_type()
00: main(): int v = a.get_result();
00: main(): v = -572662307 = 0xdddddddd;
00: main(): return 0;
00: 000000F411AFF698: task::~task(): test on coro_.done()
00: 000000F411AFF698: task::~task(): !coro.done()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       4       0       4
00: fin 4       4       0       1
00: --------------------------------------------------------
00: Waiting 1000 milliseconds before exiting
PS ...>
```

Notice the value v = -572662307 = 0xdddddddd.
The get_result() function accesses memory that has already been released
and re-initialized by the Windows operating system.
Notice that coroutine1() co-returned the correct value, i.e. wrote 4 to the promise_type object in 
the coroutine1 state object.

The reader is referred to the p11X0_sn.cpp and p11X0e_sn.cpp examples.
"sn" stands for suspend_never.

### Second implementation

To (try to) avoid this problem, we can store the result in the task object instead of in the promise_type object.
This is described in the [this section](#pushing-the-result-from-the-promise_type-object-to-the-async_task-object).

File [taske_sn2.h](taske_sn2.h) contains an implementation of a 'task' class that uses eager start.
File [task_sn2.h](task_sn2.h) contains another implementation of a 'task' class that uses lazy start.

'sn2' is short for the use of suspend_never (second implementation) at the final suspend point.

```
PS ...> .\stfs-p1160e_sn2.exe
00: main(): task a = obj.coroutine1();
00: 00000206847689D0: promise_type::promise_type()
00: task::task(...)
00: coroutine1(): int v = co_await coroutine2();
00: 0000020684768B00: promise_type::promise_type()
00: task::task(...)
00: coroutine2(): int v = co_await coroutine3();
00: 0000020684768C30: promise_type::promise_type()
00: task::task(...)
00: coroutine3(): int v = co_await coroutine4();
00: 0000020684768D60: promise_type::promise_type()
00: task::task(...)
00: coroutine4(): co_await are1;
00: main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));
00: main(): are1.resume();
00: coroutine4(): co_return 1;
00: 0000020684768C60: task::~task(): test on coro_.done()
00: 0000020684768C60: task::~task(): !coro.done()
00: coroutine3(): co_return 2;
00: 0000020684768B30: task::~task(): test on coro_.done()
00: 0000020684768B30: task::~task(): !coro.done()
00: coroutine2(): co_return 3;
00: 0000020684768A00: task::~task(): test on coro_.done()
00: 0000020684768A00: task::~task(): !coro.done()
00: coroutine1(): co_return 4;
00: 00000206847689D0: promise_type::~promise_type()
00: 0000020684768B00: promise_type::~promise_type()
00: 0000020684768C30: promise_type::~promise_type()
00: 0000020684768D60: promise_type::~promise_type()
00: main(): int v = a.get_result();
00: main(): v = 4 = 0x4;
00: main(): return 0;
00: 0000008CDFBCF7E8: task::~task(): test on coro_.done()
00: 0000008CDFBCF7E8: task::~task(): !coro.done()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       4       0       4
00: fin 4       4       0       1
00: --------------------------------------------------------
00: Waiting 1000 milliseconds before exiting
PS ...>
```

This implementation is both functionally correct and does not have any memory leaks.

The reader is referred to the p11X0_sn2.cpp and p11X0e_sn2.cpp examples with X > 5.
'sn2' stands for the suspend_never variant using the task object to store the result.

## Using a custom final_awaiter type, await_ready() returns false

In this section we will investigate 3 custom final_awaiter types,
where await_suspend() returns either

* void
* bool, so that the coroutine may conditionally suspend at the final_suspend_point
* std::coroutine_handle<>

void, bool and std::coroutine_handle<> are the 3 possible return types of await_suspend().
Class async_task in [async_task.h](../../include/corolib/async_task.h)
can be tailored to use any of these 3 variants.

Unless otherwise mentioned,
the return_value() (and return_void()) function will no longer call resume():

```c++
    void return_value(int v)
    {
        value = v;
    }
```

Instead, resume() will be called from within the final_awaiterX::await_suspend() 
or just after it in case final_awaiterX::await_suspend() has return type std::coroutine_handle<>
(see https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer for this last case).

### await_suspend() returns void

File [taske_void.h](taske_void.h) contains an implementation of a 'task' class that uses eager start.
File [task_void.h](task_void.h) contains another implementation of a 'task' class that uses lazy start.

'void' stands for the use a final_awaiter type with await_ready() returning false and await_suspend() returning void.

The custom final_awaiter type is defined as follows:

```c++
    struct final_awaiter {
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<> h) noexcept {
            if (h.promise().m_continuation)
                h.promise().m_continuation.resume();
        }
        void await_resume() noexcept { }
    };
    
    auto final_suspend() noexcept {
         return final_awaiter1{};
    }
```

To use this final_awaiter variant, set the compiler directives
USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL 
and USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE both to 0 in [async_task.h](../../include/corolib/async_task.h).

The reader is referred to the p12X0_void.cpp and p12X0e_void.cpp examples.

### await_suspend() returns bool

#### First implementation

```c++
    struct final_awaiter {
        bool await_ready() const noexcept { return false; }
        bool await_suspend(std::coroutine_handle<> h) noexcept {
            return !h.promise().m_ready;
        }
        void await_resume() noexcept { }
    };
    
    auto final_suspend() noexcept {
         return final_awaiter{};
    }
```

The final_awaiter1 object is a custom type that suspends the coroutine
if the result is not 'ready' or 'present' at the final suspend point.
This should not happen; if it does, there is probably an error in the application flow.

h.promise().m_ready should always return true, so await_suspend() will return false.
This approach is very similar to using suspend_never. 

Note that in this first implementation, there is no resume() call in final_awaiter1::await_suspend().
Resumption still has to take place from the return_value() or return_void() function.

This first implementation is not discussed any further.

#### Second implementation

File [taske_bool.h](taske_bool.h) contains an implementation of a 'task' class that uses eager start.
File [task_bool.h](task_bool.h) contains another implementation of a 'task' class that uses lazy start.

'bool' stands for the use a final_awaiter type with await_ready() returning false and await_suspend() returning a bool.

The second implementation looks as follows:

```c++
    struct final_awaiter1 {
        bool await_ready() noexcept {
            return false;
        }
        bool await_suspend(coroutine_handle<promise_type> h) noexcept {
            auto& promise = h.promise();
            if (promise.ready.exchange(true, std::memory_order_acq_rel)) {
                h.promise().continuation.resume();
            }
            // This function must return true.
            // Otherwise the coroutine will run to an end and the coroutine frame will be deleted.
            // The call of get_result() will retrieve the result from released memory.
            return true;
        }
        void await_resume() noexcept {}
    };
```

The transformed code may look as follows:

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* __coroutine1_resume(__coroutine_state* s) {
        auto* state = static_cast<async_task_int_state*>(s);
        ...
        ...
            // co_return v + 1;
            state->_promise.return_value(v + 1);
            goto final_suspend_point;
        }
        catch (...) {
            state->_promise.unhandled_exception();
        }
    final_suspend_point:
        // co_await state->_promise.final_suspend();
        if (!state->_fa.await_ready()) {
            state->_suspend_point = N;    // N depends on the previous number of suspend points
            state->_resume = nullptr;     // mark as final suspend-point: done() will return true
            if (!state->_fa.await_suspend(
                    std::coroutine_handle<__coroutine1_promise_t>::from_promise(state->_promise)))
                goto end;
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }
        state->__fa.await_resume();
    end:
        delete state;
        return static_cast<__coroutine_state*>(std::noop_coroutine().address());
    }
```

This again leads to problems when calling get_result() from main():
at that point, the coroutine state (with the promise_type object inside) has been deleted
and get_result() may/will return random results (if not worse).

To use final_awaiter1, set the compiler directives
USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL to 1
and USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE to 0 in [async_task.h](../../include/corolib/async_task.h).

The reader is also referred to the p13X0_bool.cpp and p13X0e_bool.cpp examples.

#### Multi-threaded applications

The approach described in the previous two sections avoided the memory leaks from the original implementation.
However, applications where the resumption takes place from a separate thread are now unstable.
Making them behave correctly would require the use of std::atomic (and possibly also mutexes)
for all data members that can be accessed from multiple threads.

This approach has not been implemented yet.
One reason is that the preferred implementation should avoid the overhead
of std::atomic and mutexes in single-threaded applications.

### Using symmetric transfer: await_suspend() returns std::coroutine_handle<>

For an introduction to symmetric transfer, the reader is referred to
https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer

This approach should also avoid the use of std::atomic variables.

In the symmetric transfer approach, 
task::promise_type::final_awaiterX::await_resume()
no longer calls std::coroutine_handle<TYPE>::resume() on a coroutine handle.
Instead, final_awaiterX::await_suspend() returns a coroutine_handle.

File [taske_coroutine_handle.h](taske_coroutine_handle.h) contains an implementation of a 'task' class that uses eager start.
File [task_coroutine_handle.h](task_coroutine_handle.h) contains another implementation of a 'task' class that uses lazy start.

'coroutine_handle' stands for the use a final_awaiter type with await_ready() returning false and await_suspend() returning a coroutine_handle.

Struct final_awaiter is defined as follows:

```c++
    struct final_awaiter {
        bool await_ready() const noexcept { return false; }
        std::coroutine_handle<> await_suspend(handle_type_own h) noexcept {
            if (h.promise().m_continuation)
                return h.promise().m_continuation;
            else
                return std::noop_coroutine();
        }
        void await_resume() noexcept { }
    };
    
    auto final_suspend() noexcept {
         return final_awaiter2{};
    }
```

The transformed code may now look as follows:

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* __coroutine1_resume_(__coroutine_state* s) {
        auto* state = static_cast<async_task_int_state*>(s);
        ...
        ...
            // co_return v + 1;
            state->_promise.return_value(v + 1);
            goto final_suspend_point;
        }
        catch (...) {
            state->_promise.unhandled_exception();
        }
    final_suspend_point:
        // co_await state->_promise.final_suspend();
        if (!state->_fa.await_ready()) {
            state->_suspend_point = N;    // N depends on the previous number of suspend points
            state->_resume = nullptr;     // mark as final suspend-point: done() will return true
            std::coroutine_handle<> h = state->_fa.await_suspend(
                    std::coroutine_handle<__coroutine1_promise_t>::from_promise(state->_promise));
            return static_cast<__coroutine_state*>(h.address());
        }
        state->__fa.await_resume();
        
        delete state;
        return static_cast<__coroutine_state*>(std::noop_coroutine().address());
    }
```

The std::coroutine_handle<> returned by final_awaiter2::await_suspend()
is cast to a pointer to a __coroutine_state object.
Function __coroutine1_resume() returns this pointer to its caller, which is std::coroutine_handle::resume().
This library function then calls the "virtual" __resume function that "belongs" to the __coroutine_state object:

```c++
    void resume() const {
        __coroutine_state* s = state_;
        do {
            s = s->__resume(s);
        } while (s != &__coroutine_state::__noop_coroutine);
    }
```

This loop ends when the __resume() function returns a "null pointer," which corresponds to the address of the
__coroutine_state::__noop_coroutine object.

To use final_awaiter2 in corolib, set the compiler directives
USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL to 0
and USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE to 1 in [async_task.h](../../include/corolib/async_task.h).

The reader is also referred to the p14X0_coroutine_handle.cpp and p14X0e_coroutine_handle.cpp examples.

Notice that the implementation of await_suspend() in struct final_awaiter2
is very close to the await_suspend() implementation in final_awaiter0.

## Using a custom final_awaiter type, await_ready() returns true

If await_ready() returns true, await_suspend() is not called.
This means that the 3 possible variants of await_suspend() do not matter.
The example below uses the await_suspend() variant that returns bool, but
any of the two other variants could have been used as well.
The implementation of this function can be left empty.

File [taske_bool2.h](taske_bool.h) contains an implementation of a 'task' class that uses eager start.
File [task_bool2.h](task_bool.h) contains another implementation of a 'task' class that uses lazy start.

bool2 stands for the use of final_awaiter type with await_ready() returning true and await_suspend() returning a bool.

The implementation of final_awaiter can look as follows:

```c++
    struct final_awaiter {
        final_awaiter(promise_type* p1)
            : p(p1) {
        }

        bool await_ready() noexcept {
            return true;
        }
        bool await_suspend(coroutine_handle<promise_type> h) noexcept {
            // This function is not called
            return true;    // Could have been false
        }
        void await_resume() noexcept {
            if (p)
                if (p->continuation)
                    p->continuation.resume();
        }
    };
```

From a semantic point of view, it seems to be more logical to have await_ready() return true
instead of false: if the coroutine if not "ready" at the final suspend point, then when will it be ready?
Also, function await_suspend() is used to resume the previous coroutine (coroutineN if the current coroutine is coroutineN+1).
It may make more sense to resume the previous coroutine from a function called await_resume() instead of await_suspend().
In practice, at the final suspend point, await_ready() means: do we suspend or resume the coroutine at this point?

An important difference between await_suspend() and await_resume() (apart from the return type) is that
await_suspend() takes a coroutine_handle<promise_type> argument, which has been used above
to find the handle to the coroutine that has to be resumed.
Because await_resume() does not take an argument, the information needed to resume the previous coroutine must already be stored
when we reach await_resume().
This explains passing a promise_type pointer to the final_awaiter constructor.
A similar situation occurs when using return_value(TYPE) or return_void() to resume the previous coroutine.

Also, because the coroutine will run to the end, we end up with a situation similar to the return
of std::suspend_never. The return value of the coroutine must be stored in the task object.

In addition, the task object must not destroy the coroutine state by calling destroy():
the destructor of class task can be left empty.

## How about resuming from await_ready()?

Up till now, we have seen that coroutibeN+1 can resume coroutineN from the functions return_value(TYPE), return_void;
final_awaiter::await_suspend(coroutine_handle<promise_type>) or final_awaiter::await_resume().
There is nothing that forbids to do this from final_awaiter::await_ready() and then have an empty implementation
of the other two final_awaiter functions.

This alternative will not be explored further in this document.

## Pushing the result from the promise_type object to the async_task object

To remedy the problem that the coroutine state object (with the embedded promise_type object)
was destroyed before get_result() could retrieve the result,
I experimented with an approach where a coroutine promise_type object
"pushes" the result to the coroutine return object, so that afterwards
the coroutine can safely use a final awaiter object that allows it to proceed at the final suspend point.

The lifetime of the coroutine's "intended" return value is the same as that of the coroutine return object
because the intended value is now a data member of the coroutine return object.
In my opionion this is also the logical place of this return value 
(it is closer to the return behavior of an ordinary function).
The lifetime of the promise_type object can be longer or shorter than that of the coroutine return
object it created. However, this has become irrelevant.

To have the promise_type object "push" the result to the coroutine return object, 
set USE_RESULT_FROM_COROUTINE_OBJECT to 1 in [async_task.h](../../include/corolib/async_task.h).

For this to be possible, some additional "linking" between the promise_type object
and its coroutine return object has to be in place first.
The reason is that a promise_type object does normally not have any pointer or reference
to the coroutine return object it creates when calling get_return_object().

To enable this additional linking, set USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN to 1 
in [async_task.h](../../include/corolib/async_task.h).
USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN can also be used for other purposes
than for placing the result in the coroutine return object. More in particular,
it is used for tracking which object (coroutine return object or promise_type object)
goes out of scope first.

USE_RESULT_FROM_COROUTINE_OBJECT = 1 requires USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN = 1,
but not the other way around.

## Overview of the settings

Please see the comments in [async_task.h](../../include/corolib/async_task.h).

For an overview of the study examples, the reader is referred to the [README](./README.md) file.
