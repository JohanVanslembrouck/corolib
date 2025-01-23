# Avoiding memory leaks

## Introduction

The original implementation of async_task has some memory leaks because not all coroutine frames
(with their "embedded" promise_type objects) were released. The following explains the causes of the problem
and how it is solved.

The reader is also referred to ../examples/study-final_suspend that explains the problem with simpler task class implementations
and both for eager and lazy start coroutines. The latter are not considered in this post.

## Example

The explanation below will use the following coroutine application.

```c++
    auto_reset_event are1;

    async_task<int> coroutine3() {
        co_await are1;        // Will suspend here and pass control to coroutine2()
                              // Will be resumed from main() and run to completion
        co_return 1;
    }

    async_task<int> coroutine2() {
        int v = co_await coroutine3();   // Will suspend here and pass control to coroutine1()
                                         // Will be resumed from coroutine3() and run to completion
        co_return v+1;
    }

    async_task<int> coroutine1() {
        int v = co_await coroutine2();   // Will suspend here and pass control to main()
                                         // Will be resumed from coroutine2() and run to completion
        co_return v+1;
    }

    int main() {
        async_task<int> a = coroutine1();
        are1.resume();                // Will resume coroutine3()
        int v = a.get_result();
        return 0;
    }
```

For a full example the reader is referred to e.g. ../examples/tutorial/p1100-auto_reset_event.cpp.
For the definition of async_task, the reader is referred to ../include/corolib/async_task.h.

async_task<TYPE> uses eager start.
Therefore coroutines returning an async_task<TYPE> object can be started from main().
There is no need to apply co_await to any coroutine to get it beyond its initial suspend point
(see also the next code fragment).

A first high-level transformation of coroutine1 may look as follows:

```c++
    async_task<int> coroutine1() {
        async_task_int_state* __state = new async_task_int_state{};
        save_frame_pointer(__state);
        async_task<int> __return = __state->_promise.get_return_object();
        
        // Initial suspend point
        co_await __state->_promise.initial_suspend();

        try {
            // Original code
            int v = co_await coroutine2();
            co_return v + 1;
        }
        catch (...) {
            __state->_promise.unhandled_exception();
        }

        // Final suspend point
        co_await __state->_promise.final_suspend();
    }
```

The next transformation step will most probably split coroutine1() into a "ramp function" and a "resume function".
See https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform for more information.

The ramp function has the same signature as the original coroutine:
```c++
    async_task<int> coroutine1()
```
The resume function has the following signature: 
```c++
    __coroutine_state* coroutine1(__coroutine_state*)
```

## Original approach: final_suspend returns std::suspend_always

In the original implementation of async_task::promise_type, the function return_value() is
implemented as follows:

```c++
    void return_value(int v)
    {
        m_value = v;
        m_ready = true;
        m_continuation.resume(); // We resume the co_awaiting coroutine from here
    }
```

Function final_suspend() returns std::suspend_always:

```c++
    auto final_suspend() noexcept {
        return std::suspend_always{};
    }
```

where suspend_always can be defined as follows:

```c++
struct suspend_always {
     constexpr suspend_always() noexcept = default;
     constexpr bool await_ready() const noexcept { return false; }
     constexpr void await_suspend(coroutine_handle<>) const noexcept {}
     constexpr void await_resume() const noexcept {}
};
```

The part of the coroutine1 resume function (from the co_return statement till the end)
may look as follows:

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* coroutine1(__coroutine_state* s) {
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
        if (!state->_fa.await_ready()) {    // await_ready() always returns false in case of suspend_always
            state->_suspend_point = N;        // N depends on the previous number of suspend points
            state->_resume = nullptr;         // mark as final suspend point: done() will return true
            state->_fa.await_suspend(
                std::coroutine_handle<__coroutine1_promise_t>::from_promise(state->_promise));
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }
        // We will never reach this point!
        state->__fa.await_resume();
        
        delete state;
        return static_cast<__coroutine_state*>(std::noop_coroutine().address());
    }
```

The coroutine unconditionally suspends at the final suspend point:
the coroutine state and the promise_type object inside it are not (yet) destroyed.
This way, the calling coroutine can still access the result of the called coroutine via the coroutine return object,
e.g. by calling a library function get_result().
Such an access is necessary in the main() function that cannot call co_await.

However, there is no resume() call anywhere that will eventually force the coroutine to proceed to the end:
this will lead to memory leaks.

To (try to) avoid this, the destructor of the coroutine return object must call the destroy() function as follows:

```c++
    ~async_task_base()
    {
        if (m_coro_handle) {     // Does the coroutine_handle still contain a valid pointer to the coroutine frame/state?
           if (m_coro_handle.done()) {     // Has the coroutine reached the final suspend point (and cleared the "resume" function pointer)?
               m_coro_handle.destroy();    // Call "destroy" function
           }
       }
    }
```

With the implementation of return_value(int val) above,
coroutineN+1 resumes its calling coroutine coroutineN in its co_return statement
(that translates to return_value(...)).

In the resume flow of the example application (see first section),
the coroutine return object that is returned by coroutineN+1 and co_await-ed upon by coroutineN,
will go out of scope *before* coroutine coroutineN+1 has reached its final suspend point.
In other words, the destructor of this coroutine return object
will be called *before* coroutine coroutineN+1 has reached its final suspend point.
Consequently, m_coro_handle.done() returns false and destroy() will not be called.
Calling destroy() anyway would lead to program misbehavior.

## Using a custom final_awaiter

Instead of final_suspend() returning std::suspend_always, what would be the result if final_suspend() returns
std::suspend_never instead? This alternative is not discussed here.
The reader is referred to the p11X0_sn.cpp and p11X0e_sn.cpp examples in ./examples/study-final_suspend
(where "sn" stands for suspend_never). Instead, we will investigate 3 custom final_awaiter types, where await_suspend() returns ...

* bool, so that the coroutine may conditionally suspend at the final_suspend_point
* std::coroutine_handle<>
* void

Note that void, bool and std::coroutine_handle<> are the three possible return types of await_suspend().

### await_suspend() returns bool

Let's try to eliminate the problem by defining a custom final_awaiter1 type that is defined and used as follows:

```c++
    struct final_awaiter1 {
        bool await_ready() const noexcept { return false; }
        bool await_suspend(std::coroutine_handle<> h) noexcept {
            return !h.promise().m_ready;
        }
        void await_resume() noexcept { }
    };
    
    auto final_suspend() noexcept {
         return final_awaiter1{};
    }
```

The final_awaiter1 object is a custom type that suspends the coroutine
if the result has not arrived at the final suspend point.
This should not happen; if it does, there is probably an error in the application flow.

h.promise().m_ready should always return true, so await_suspend() will return false.
This approach is very similar to using suspend_never. 

The transformed code may look as follows:

```c++
    using __coroutine1_promise_t = std::coroutine_traits<async_task>::promise_type;

    __coroutine_state* coroutine1_resume(__coroutine_state* s) {
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
            state->_resume = nullptr; // mark as final suspend-point: done() will return true
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

However, this leads to problems when calling get_result() from main():
at that point, the coroutine frame/state (with the promise_type object inside) has been deleted
and get_result() may/will return random results (if not worse).

To use the adapted final_awaiter1, set the compiler directive USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL to 1 in async_task.h

#### Pushing the result from the promise_type object to the async_task object

To remedy the problem described in the previous section,
I experimented with an approach where a coroutine promise_type object
"pushes" the result to the coroutine return object, so that afterwards
the coroutine can safely use a final_awaiter1 object that allows it to proceed at the final suspend point.

The lifetime of the coroutine's "intended" return value is the same as that of the coroutine return object
because the intended value is now a data member of the coroutine return object.
In my opionion this is also the logical place of this return value (it is closer to the return behavior of an ordinary function).
The lifetime of the promise_type object can be longer or shorter than that of the coroutine return
object it created; this has become irrelevant.

To have the promise_type object "push" the result to the coroutine return object, 
set USE_RESULT_FROM_COROUTINE_OBJECT to 1.

For this to be possible, some additional "linking" between the promise_type object
and its coroutine return object has to be in place first.
The reason is that a promise_type object does normally not have any pointer or reference
to the coroutine return object it creates when calling get_return_object().

To enable this additional linking, set USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN to 1.

#### Multi-threaded applications

The approach described in the previous two sections avoided the memory leaks from the original implementation.
However, applications where the resumption took place from a separate thread are now very unstable.
Making them behave correctly would require the use of std::atomic (and possibly also mutexes)
for all data members that can be accessed from multiple threads.

This approach has not been implemented yet.
One reason is that the implementation should avoid the overhead
of std::atomic and mutexes in single-thread applications.

### Using symmetric transfer: await_suspend() returns std::coroutine_handle<>

For an introduction to symmetric transfer, the reader is referred to
https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer

This approach should also avoid the use of std::atomic variables.

In the symmetric transfer approach, the (corolib) library code will no longer explicitly call resume()
from async_task. Intead, the library code will return a coroutine_handle() to the generated code.
The generated code will then call resume() on the returned handle.

The return_value() (and return_void()) function will no longer call resume():

```c++
    void return_value(int v)
    {
        m_value = v;
        m_ready = true;
    }
```

The final_awaiter2 type is defined and used as follows:

```c++
    struct final_awaiter2 {
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

    __coroutine_state* coroutine1(__coroutine_state* s) {
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

### await_suspend() returns void

Looking at the implementation of await_suspend() in struct final_awaiter2 (see previous section),
it is straight-forward to replace it with the following implementation:

```c++
    struct final_awaiter0 {
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

In this implementation, await_suspend() has void as return type instead of std::coroutine_handle<>
and it has to resume h.promise().m_continuation.

## Overview of the settings

Please see the comments in async_task.h.
