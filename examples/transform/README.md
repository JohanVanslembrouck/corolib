# Transform

## Introduction

This directory contains examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

The examples are very heavily inspired by the article https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform 
and the code in https://godbolt.org/z/xaj3Yxabn

Example p0100trf.cpp contains the code from https://godbolt.org/z/xaj3Yxabn.
I added a main() function to make an executable.
All modifications to the original code have been commented with // JVS.
The program does not do anything useful (yet).

Starting from p02XX, the following convention is used:

* p0200.h contains the task coroutine type that is used in all p02XX.cpp and p02XXtrf.cpp files.
* p02XX.cpp contains a coroutine program that is transformed and compiled by the C++ compiler.
* p02XXtrf.cpp contains the same program but with manually transformed coroutines.

From p02XX.cpp two executables are produced:

* p02XX(.exe) is produced by using #include \<coroutine> (which is the coroutine header file that comes with the compiler).
* p02XXlb(.exe) is produced by using #include "lbcoroutine.h" (which is the coroutine header file written by Lewis Baker).

Finally

* p02XXtrf(.exe) is produced by using #include "lbcoroutine.h".

# Dealing with the 3 variants of await_suspend(std::coroutine_handle<>)

The await_suspend(std::coroutine_handle<>) function can return
* void
* a bool
* a std::coroutine_handle<>

https://blog.panicsoftware.com/co_awaiting-coroutines/ shows the kind of code a compiler generates for these three variants.
The code in the just mentioned article has been used as a guidance to tailor the manually transformed code 
in the p02X0-g.h and p02X0-h.h files.
(It is not applicable to the p02X0-h.h files.)

A high-level view of the transformation of a coroutine using an await_suspend() function that returns

* void can be found in ../corolab/p0403at.cpp (and ../corolab/p0403dt.cpp)
* a bool can be found in ../corolab/p0403bt.cpp
* a std::coroutine_handle<> can be found in ../corolab/p0403ct.cpp

The transformation does not only depend on the content of the coroutine,
but also on the return type of the await_suspend() function.

### await_suspend(std::coroutine_handle<>) returns void

The file p0200.h compiled with directive AWAIT_SUSPEND_RETURNS_VOID = 1
defines a coroutine type with an awaiter type whose await_suspend() function returns void:

```cpp
    struct awaiter {
        explicit awaiter(std::coroutine_handle<promise_type> coro) noexcept
            : m_coroutine(coro)
        { }
        bool await_ready() noexcept {
            return m_coroutine.promise().m_ready;
        }
        void await_suspend(std::coroutine_handle<> awaiting) noexcept {
            m_coroutine.promise().m_awaiting = awaiting;
        }
        int await_resume() {
            return m_coroutine.promise().m_value;
        }
    private:
        std::coroutine_handle<promise_type> m_coroutine;
    };
```

This variant of p0200.h is used in the p020X.cpp and p21X.cpp program files.

The files p0200-g.h and p0200-h.h contain the transformation 
of the g and h coroutines used in the program files just mentioned.

### await_suspend(std::coroutine_handle<>) returns bool

The file p0200.h compiled with directive AWAIT_SUSPEND_RETURNS_BOOL = 1
defines a coroutine type with an awaiter type whose await_suspend() function returns a bool:

```cpp
   struct awaiter {
        explicit awaiter(std::coroutine_handle<promise_type> coro) noexcept
            : m_coroutine(coro)
        { }
        bool await_ready() noexcept {
            return m_coroutine.promise().m_ready;
        }
        bool await_suspend(std::coroutine_handle<> awaiting) noexcept {
            m_coroutine.promise().m_awaiting = awaiting;
            return !m_coroutine.promise().m_ready;
        }
        int await_resume() {
            return m_coroutine.promise().m_value;
        }
    private:
        std::coroutine_handle<promise_type> m_coroutine;
    };

```

This variant of p0200.h is used in the p022X.cpp and p23X.cpp program files.

The files p0220-g.h and p0220-h.h contain the transformation 
of the g and h coroutines used in the program files just mentioned.

### await_suspend(std::coroutine_handle<>) returns std::coroutine_handle<>

The file p0200.h compiled with directive AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1
defines a coroutine type with an awaiter type whose await_suspend() function returns 
a std::coroutine_handle<>:

```cpp
   struct awaiter {
        explicit awaiter(std::coroutine_handle<promise_type> coro) noexcept
            : m_coroutine(coro)
        { }
        bool await_ready() noexcept {
            return m_coroutine.promise().m_ready;
        }
        std::coroutine_handle<> await_suspend(std::coroutine_handle<> awaiting) noexcept {
            m_coroutine.resume();
            return awaiting;
        }
        int await_resume() {
            return m_coroutine.promise().m_value;
        }
    private:
        std::coroutine_handle<promise_type> m_coroutine;
    };
```

This variant of p0200.h is used in the p024X.cpp and p25X.cpp program files.

The files p0240-g.h and p0240-h.h contain the transformation 
of the g and h coroutines used in the program files just mentioned.

## Notes

### The MSVC compiler does not see the noexcept operator

The MSVC compiler (Visual Studio 2022) produces the following error:

    C5231   the expression 'co_await promise.final_suspend()' must be non-throwing 

on the following lines

```cpp
inline task::promise_type::final_awaiter task::promise_type::final_suspend() noexcept {     // here
    return {};
}}
```

```cpp
        auto final_suspend() noexcept {                                 // here
            print(PRI2, "task::promise_type::final_suspend()\n");
#if USE_FINAL_AWAITER
            return final_awaiter{ *this };
#else
            return std::suspend_always{};
#endif
        }
```

when lbcoroutine.h is included. However, there is clearly a noexcept operator.

The g++ 11.4.0 compiler on Ubuntu 22.04 LTS does not complain and produces executables.

### g++ 11.4.0 produces a warning for 'offsetof'

Example:

```cpp
path_to_corolib/corolib/examples/transform/lbcoroutine.h:167:26: warning: ‘offsetof’ within non-standard-layout type ‘std::coroutine_handle<task::promise_type>::state_t’ {aka ‘__coroutine_state_with_promise<task::promise_type>’} is conditionally-supported [-Winvalid-offsetof]
  167 |                 offsetof(state_t, __promise));
      |                          ^
```

The behavior is correct.
