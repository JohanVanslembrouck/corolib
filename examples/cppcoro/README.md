# CppCoro - A coroutine library for C++

Please see https://github.com/lewissbaker/cppcoro for the original README.md.

This directory contains the complete include/ and lib/ directories of the cppcoro project, but with the following modifications:

* All occurrences of "experimental" have been removed to use the C++20 standard.

* async_scope.hpp and task.h: declared a function as "noexcept" in each file.

* Modifications have been made to the following files to allow the use of the "net" classes from corolib.

  * win32_overlapped_operation.hpp: allow the corolib library to register its completion handler. All additionals have been marked with " Addition for corolib".

  * socket_accept_operation.hpp, socket_connect_operation.hpp, socket_disconnect_operation.hpp, socket_recv_from_operation.hpp, socket_recv_operation.hpp, socket_send_operation.hpp, socket_send_to_operation.hpp: allow corolib to access functions try_start() (and some others).

The directories examples-cc (cc = cppcoro) and examples-cl (cl = corolib) contain some client-server examples. 
Directory examples-cc uses cppcoro coroutines, while examples-cl uses corolib coroutines.
Files with the same name in both directory contain the same functionality,
but implemented with a different coroutine implementation.
