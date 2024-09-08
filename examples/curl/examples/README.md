# Using curl with corolib

Note: The inspiration for these examples is the video 
"What We’ve Been Awaiting For? - How to Build a C++ Coroutine Type - Hana Dusíková - C++Now 2024"
(https://www.youtube.com/watch?v=78nwm9EP23A).
However, the starting point for the implementation are original curl examples, as described next.

This directory contains (currently only two) source code files 
from https://github.com/curl/curl/tree/master/docs/examples with two minor modifications:
The .c file names have been renamed to .cpp files, and an existing URL (currently taken from the video) is used.
This leads to the following files/ multi-single.cpp and multi-double.cpp.

From these files, a class variant was made: 
the original code in main() has been split into small functions and placed in a class CoCurl.
This leads to multi-single-class.cpp and multi-double-class.cpp.

This code in these latter files is then used as a starting point for implementing the coroutine variant.
Files multi-single-coroutine.cpp and multi-double-coroutine.cpp demonstrate the use of curl with corolib.

After building, start any of the executables.
