# Transform

This directory contains examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

The examples are very heavily inspired by the article https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform and the code in https://godbolt.org/z/xaj3Yxabn

Example p0100trf.cpp contains the code from https://godbolt.org/z/xaj3Yxabn.
I added a main() function to make an executable. All modifications to the original code have been marked with // JVS.
The program does not do anything useful (yet).

The following convention is used:

* p0XX0.h contains the task coroutine type that is used in the .cpp files.
* p0XXX.cpp contains a coroutine program that is transformed by the C++ compiler.
* p0XXXtrf.cpp contains the same program but with manually transformed coroutines.

From p0XXX.cpp, two executables are produced:

* p0XXX(.exe) is produced by using #include \<coroutine> (which is the coroutine header file that comes with the compiler).
* p0XXXlb(.exe) is produced by using #include "lbcoroutine.h" (which is the coroutine header file by Lewis Baker).

All three variants should exhibit the same behavior.

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

when lbcoroutine.h is included. However, there is clearly a noexcept.

The g++ 11.4.0 compiler on Ubuntu 22.04 LTS does not complain and produces executables.
