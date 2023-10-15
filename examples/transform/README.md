# Transform

This directory contains examples that illustrate how a C++ compiler may transform coroutine code 
to C++ code that can be compiled with a C++ compiler that does not support coroutines.

The examples are very heavily inspired by the article https://lewissbaker.github.io/2022/08/27/understanding-the-compiler-transform and the code in https://godbolt.org/z/xaj3Yxabn

Example p0100trf.cpp contains the code from https://godbolt.org/z/xaj3Yxabn.
I added a main() function to make an executable. All modifications to the original code have been marked with // JVS.
The program does not do anything useful (yet).

The following convention is used:

* p0XXX.cpp contains a coroutine program that #include <coroutine> (which is the coroutine header file that comes with the compiler).
* p0XXXlb.cpp is exactly the same program, but instead it #include "lbcoroutine.h" (which is the coroutine header file by Lewis Baker).
* p0XXXtrf.cpp contains the same program with manually transformed coroutines.

All three variants should exhibit the same behavior.

I will add more examples in the (near) future.
