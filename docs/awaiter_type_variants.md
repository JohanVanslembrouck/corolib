# On the many variants of awaiter types

## Introduction

There are lots of things to learn when you are new to C++ coroutines,
especially when you want to develop a coroutine library yourself:

* co_return, co_await, co_yield,
* concepts (not in the sense of C++ concepts) such as 
  * suspend and resume (in addition to call and return),
  * coroutine state or frame,
  * coroutine handle,
  * coroutine (return) type,
  * continuation,
  * eager and lazy start coroutines,
  * awaiter type, with its 3 functions await_ready(), await_suspend() and await_resume(),
  * awaitable type
* suspend_always and suspend_never,
* promise_type, with its many functions
  * get_return_object(),
  * initial_suspend(),
  * final_suspend(), 
  * return_value(), return_void(), yield_value(),
  * unhandled_exception(),
  * get_return_object_on_allocation_failure().

Starting from simple implementations, you can make them more complete (and complex).

Here, we will only consider awaiter types and initial_suspend() and final_suspend().

## initial_suspend() and final_suspend() returning std::suspend_never or std::suspend_always

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
This is a coroutinw with eager start.

The following is the definition of a coroutine with lazy staet:

```c++
class task {
public:
    class promise_type {
    public:
        // ...

        std::suspend_always initial_suspend() noexcept {  // lazy start coroutine
            return {};
        }

        std::suspend_always final_suspend() noexcept {
            return {};
        }

        // ...
    };
};
```
std::suspend_never and std::suspend_always can be defined as follows:

```c++
namespace std
{
    struct suspend_always {
        constexpr suspend_always() noexcept = default;
        constexpr bool await_ready() const noexcept { return false; }
        constexpr void await_suspend(coroutine_handle<>) const noexcept {}
        constexpr void await_resume() const noexcept {}
    };

    struct suspend_never {
        constexpr suspend_never() noexcept = default;
        constexpr bool await_ready() const noexcept { return true; }
        constexpr void await_suspend(coroutine_handle<>) const noexcept {}
        constexpr void await_resume() const noexcept {}
    };
}
```

The reader is referred to [initial_suspend](../studies/initial_suspend/README.md)
for a comparison of eager and lazy start coroutines.

In total 4 combinations are possible, because final_suspend() can also return
std::suspend_never instead of std::suspend_always as in the 2 code fragments above.

std::suspend_never and std::suspend_always are awaiter types; they define the following 3 functions:

* bool await_ready()
* void await_suspend(coroutine_handle<>)
* void await_resume()

## initial_suspend() and final_suspend() returning a user-defined type

Instead of returning std::suspend_never or std::suspend_always, the library writer can
introduce dedicated types, called e.g. initial_awaiter and final_awaiter.

```c++
struct initial_awaiter {
    bool await_ready() noexcept;
    void await_suspend(std::coroutine_handle<> h) noexcept;
    int await_resume();
};

struct final_awaiter {
    bool await_ready() noexcept;
    void await_suspend(std::coroutine_handle<> h) noexcept;
    int await_resume();
};
```
This gives 3 possible return types for initial_suspend() and for final_suspend(),
or 9 combinations in total.

## task::awaiter

The definition of task shown above is incomplete.
Class task also has to be an awaiter type, defining the 3 special await functions:

```c++
class task {
public:

    class promise_type {
    public:
        // ...

        std::suspend_never initial_suspend() noexcept {  // lazy start coroutine
            return {};
        }

        std::suspend_always final_suspend() noexcept {
            return {};
        }

        // ...
    };

    bool await_ready() noexcept;
    void await_suspend(std::coroutine_handle<>) noexcept;
    void await_resume() noexcept;   // await_resume can return another type than void

private:
    std::coroutine_handle<promise_type> coro_;
};
```

A variant style is possible, grouping the 3 functions in a dedicated awaiter type:

```c++
class task {
public:

    class promise_type {
    public:
        // ...

        std::suspend_never initial_suspend() noexcept {  // lazy start coroutine
            return {};
        }

        std::suspend_always final_suspend() noexcept {
            return {};
        }

        // ...
    };

    struct awaiter {
        explicit awaiter(std::coroutine_handle<promise_type> h) noexcept;
        bool await_ready() noexcept;
        void await_suspend(std::coroutine_handle<> h) noexcept;
        void await_resume();    // await_resume can return another type than void
    private:
        std::coroutine_handle<promise_type> coro_;
    };

    awaiter operator co_await() && noexcept;

private:
    std::coroutine_handle<promise_type> coro_;
};
```

Notice the definition of operator co_await() that returns an object of type 'awaiter'.
Class task has become an awaitable because it defines operator co_await.

The definition of class task and its promise_type uses in general 3 awaiter types
(initial_awaiter, final_awaiter and task::awaiter).

## The variants of await_suspend()

Up till now, await_suspend() only had void as return type.
But it can also return bool or std::coroutine_handle<>.

This gives 3 variants for the types initial_awaiter, final_awaiter and task::awaiter 
or 27 combinations. std::suspend_never and std::suspend_always are special (trivial)
implementations of an initial_awaiter and a final_awaiter type with await_suspend() returning void.

Taking into account std::suspend_never and std::suspend_always as dedicated initial and final awaiter types,
there are in total 75 possible combinations (5 * 5 * 3).

## How to proceed?

Are these 75 combinations equally viable and/or valuable, or do some combinations not make any sense?

Subsets of these combinations are described in several places.

* [initial_suspend](../studies/initial_suspend/README.md) describes the awaiter types returned by initial_suspend(),
more in particular in the context of eager and lazy start
* [final_suspend](../studies/final_suspend/README.md) describes the awaiter types returned by final_suspend(), and which types to use 
  * to avoid memory leaks (because a coroutine frame is not released) and 
  * to ensure that a coroutine frame is not released prematurely, i.e. before the coroutine result has been used.
* [transform](../studies/transform/README.md) describes the impact of the awaiter types on the generated C++ code.
* [async_task.h](../include/corolib/async_task.h) describes the different combinations supported by corolib.
