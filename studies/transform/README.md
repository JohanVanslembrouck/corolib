# Transform

## Introduction

The reader is referred to [awaiter type variants](../../docs/awaiter_type_variants.md) for an introduction to awaiter types.

This directory contains examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

The examples are very heavily inspired by the article https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform 
and the code in https://godbolt.org/z/xaj3Yxabn

Example p0100trf.cpp contains the code from https://godbolt.org/z/xaj3Yxabn.
I added a main() function to make an executable.
All modifications to the original code have been commented with // JVS.
The program does not do anything useful (yet).

## Include files

The source files (indirectly) include one of the following 3 header files:
* coroutine: the header file that comes with the compiler
* lbcoroutine.h: the header file originally coded by Lewis Baker (lb)
* lbcoroutinevf.h: a header file based on lbcoroutine.h but using virtual functions (vf)
for __resume and __destroy instead of function pointers

The selection between coroutine and lbcoroutine.h is done via config.h.
The selection between coroutine and lbcoroutinevf.h is done via configvf.h.
Either config.h or configvf.h is included in the source files and the selection is made in CMakeLists.txt.

## Dealing with the 3 variants of await_suspend(std::coroutine_handle<>)

The await_suspend(std::coroutine_handle<>) function can return
* void
* a bool
* a std::coroutine_handle<>

These 3 variants will be used for 2 awaiter types, task::awaiter and task::promise_type::final_awaiter,
giving 9 possible combinations in total.
Also, final_suspend() can return std::suspend_always instead of the hand-coded task::promise_type::final_awaiter.
This leads to 12 combinations.
Notice that not all combinations will be used.

The following table gives an overview of all programs:

| Source       | task::awaiter                              | task::promise_type::final_awaiter            |
| ------------ | ------------------------------------------ | -------------------------------------------- |
| p0200.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0200vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0202.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0202vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0204.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0204vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0206.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0206vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p1200.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p1204.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p1206.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 0                        |
| p0210.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 1                        |
| p0210vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 1                        |
| p0216.cpp    | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 1                        |
| p0216vf.cpp  | AWAIT_SUSPEND_RETURNS_VOID = 1             | USE_FINAL_AWAITER = 1                        |
| p0220.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 0                        |
| p0226.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 0                        |
| p1220.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 0                        |
| p1226.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 0                        |
| p0230.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 1                        |
| p0236.cpp    | AWAIT_SUSPEND_RETURNS_BOOL = 1             | USE_FINAL_AWAITER = 1                        |
| p0240.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 0                        |
| p0246.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 0                        |
| p1240.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 0                        |
| p1246.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 0                        |
| p0250.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 1                        |
| p0256.cpp    | AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 | USE_FINAL_AWAITER = 1                        |
| p0300.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p0302.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p0304.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p0306.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p1300.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p1304.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p1306.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1 |
| p0320.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0322.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0324.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0326.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0330.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0332.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0334.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0336.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p1320.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p1324.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p1326.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1 |
| p0340.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p0342.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p0344.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p0346.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p1340.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p1344.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |
| p1346.cpp    | (await_suspend() returns void)             | FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1 |

Remarks:
* USE_FINAL_AWAITER = 0: final_suspend() returns std::suspend_always
* USE_FINAL_AWAITER = 1: final_suspend() returns task::promise_type::final_awaiter; task::promise_type::final_awaiter::await_suspend returns void
* The p1XXX.cpp examples use a thread to resume the program on. For example, p1246.cpp is the threaded equivalent of p0246.cpp.
There is no manually transformed code for these threaded versions available (yet).

The following table shows the used combinations:

| task::awaiter::await_suspend returns | std::suspend_always | void (1)     | bool (1)     | std::coroutine_handle<> (1) |
| ------------------------------------ | ------------------- | ------------ | ------------ | --------------------------- |
| void                                 | p020X.cpp           | p021X.cpp    |              |                             |
|                                      |                     | p030X.cpp    | p032X.cpp    | p034X.cpp                   |
| bool                                 | p022X.cpp           | p023X.cpp    |              |                             |
| std::coroutine_handle<>              | p024X.cpp           | p025X.cpp    |              |                             |

(1) return type of task::promise_type::final_awaiter returns 

https://blog.panicsoftware.com/co_awaiting-coroutines/ shows the kind of code a compiler generates for these three variants.
The code in this article has been used as a guidance to tailor the manually transformed code 
in the p02X0-g.h and p02X0-h.h files.
(It is not applicable to the p02X0-f.h files.)

### awaiter::await_suspend variants

This section describes the p02XX series of exaamples.

The following convention is used:

* p0200.h contains the task coroutine type that is used in all p02XX.cpp and p02XXtrf.cpp files.
* p02XX.cpp contains a coroutine program that is transformed and compiled by the C++ compiler.
* p02XXtrf.cpp contains the same program but with manually transformed coroutines.

From p02XX.cpp two executables are produced:

* p02XX(.exe) is produced by using #include \<coroutine> (which is the coroutine header file that comes with the compiler).
* p02XXlb(.exe) is produced by using #include "lbcoroutine.h" (which is the coroutine header file written by Lewis Baker).

Finally

* p02XXtrf(.exe) is produced by using #include "lbcoroutine.h".

A high-level view of the transformation of a coroutine using an await_suspend() function that returns ...

* void can be found in ../corolab/p0403at.cpp (and ../corolab/p0403dt.cpp)
* a bool can be found in ../corolab/p0403bt.cpp
* a std::coroutine_handle<> can be found in ../corolab/p0403ct.cpp

The transformation does not only depend on the content of the coroutine,
but also on the return type of the await_suspend() function.

#### await_suspend(std::coroutine_handle<>) returns void

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

The files p0200-f.h, p0200-g.h and p0200-h.h contain the transformation 
of the f, g and h coroutines used in the program files just mentioned.

#### await_suspend(std::coroutine_handle<>) returns bool

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

#### await_suspend(std::coroutine_handle<>) returns std::coroutine_handle<>

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

### task::promise_type::final_awaiter::await_suspend variants

This section describes the p03XX series of exaamples.

The naming convention is the same as the one in the p02XXX series (see higher).

awaiter::await_suspend always returns void. Instead, final_awaiter::await_suspend will have 3 variants.

task::promise_type::return_value will *not* resume the awaiting coroutine in case FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1.
In this case the coroutine will be resumed from the final suspend section.

#### await_suspend(std::coroutine_handle<>) returns void

The file p0300.h compiled with directive FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID = 1
defines a coroutine type with an final_awaiter type whose await_suspend() function returns void:

```cpp
    struct final_awaiter {
        bool await_ready() const noexcept {
            return false;
        }
        void await_suspend(coro_handle h) noexcept {
        }
        void await_resume() noexcept {
        }

    };
```

This variant of p0300.h is used in the p030X.cpp program files.

#### await_suspend(std::coroutine_handle<>) returns bool

The file p0300.h compiled with directive FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL = 1
defines a coroutine type with an final_awaiter type whose await_suspend() function returns bool:

```cpp
    struct final_awaiter {
        bool await_ready() const noexcept {
            return false;
        }
        bool await_suspend(coro_handle h) noexcept {
            return !h.promise().m_ready;
        }
        void await_resume() noexcept {
        }

    };
```

This variant of p0300.h is used in the p032X.cpp program files.

#### await_suspend(std::coroutine_handle<>) returns std::coroutine_handle<>

The file p0300.h compiled with directive FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1
defines a coroutine type with an final_awaiter type whose await_suspend() function returns void:

```cpp
    struct final_awaiter {
        bool await_ready() const noexcept {
            return false;
        }
        std::coroutine_handle<> await_suspend(coro_handle h) noexcept {
            if (h.promise().m_awaiting)
                return h.promise().m_awaiting;
            else
                return std::noop_coroutine();
        }
        void await_resume() noexcept {
        }

    };
```

This variant of p0300.h is used in the p034X.cpp program files.

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
path_to_corolib/corolib/examples/transform/lbcoroutine.h:167:26: warning: �offsetof� within non-standard-layout type �std::coroutine_handle<task::promise_type>::state_t� {aka �__coroutine_state_with_promise<task::promise_type>�} is conditionally-supported [-Winvalid-offsetof]
  167 |                 offsetof(state_t, __promise));
      |                          ^
```

The behavior is correct.
