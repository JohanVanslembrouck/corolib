# Transform

This directory contains examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

The examples are very heavily inspired by the article https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform 
and the code in https://godbolt.org/z/xaj3Yxabn

Example p0100trf.cpp contains the code from https://godbolt.org/z/xaj3Yxabn.
I added a main() function to make an executable.
All modifications to the original code have been marked with // JVS.
The program does not do anything useful (yet).

The following convention is used:

* p0XX0.h contains the task coroutine type that is used in the .cpp files in a certain ¨range¨ (see below).
* p0XXX.cpp contains a coroutine program that is transformed by the C++ compiler.
* p0XXXtrf.cpp contains the same program but with manually transformed coroutines.

From p0XXX.cpp, two executables are produced:

* p0XXX(.exe) is produced by using #include \<coroutine> (which is the coroutine header file that comes with the compiler).
* p0XXXlb(.exe) is produced by using #include "lbcoroutine.h" (which is the coroutine header file by Lewis Baker).

All three variants should exhibit the same behavior.

An await_suspend() function can return
* a void
* a bool
* a std::coroutine_handle<>

The file p0200.h defines a coroutine type with an awaiter type whose await_suspend() function returns a void:

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

p0200.h is used in the p020X.cpp and p21X.cpp program files.

The files p0200-f.h, p0200-g.h and p0200-h.h contain the transformation 
of the f, g and h coroutines used in the program files just mentioned.

The file p0220.h defines a coroutine type with an awaiter type whose await_suspend() function returns a bool:

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

p0220.h is used in the p022X.cpp and p23X.cpp program files.

The files p0220-f.h, p0220-g.h and p0220-h.h contain the transformation 
of the f, g and h coroutines used in the program files just mentioned.

The file p0240.h defines a coroutine type with an awaiter type whose await_suspend() function returns 
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

p0240.h is used in the p024X.cpp and p25X.cpp program files.

The files p0240-f.h, p0240-g.h and p0240-h.h contain the transformation 
of the f, g and h coroutines used in the program files just mentioned.

The transformation does not only depend on the content of the coroutine,
but also on the return type of each await_suspend() function.

A high-level view of the transformation of a coroutine using an await_suspend() function
that returns
* a void can be found in ../corolab/p0403at.cpp (and ../corolab/p0403dt.cpp)
* a bool can be found in ../corolab/p0403bt.cpp
* a std::coroutine_handle<> can be found in ../corolab/p0403ct.cpp


Note that the MSVC compiler (Visual Studio 2022) produces the following error:

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
