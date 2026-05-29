# CppCoro - A coroutine library for C++

Please see https://github.com/andreasbuhr/cppcoro for the original repository and README.md.

This directory contains the complete include/ and lib/ directories of the cppcoro project, but with the following modifications:

* Modifications have been made to the following files to allow the use of the "net" classes from corolib.

  * in folder include/cppcoro/detail: win32_overlapped_operation.hpp and linux_async_operation.hpp: allow the corolib library to register its completion handler. All additions have been marked with "// Addition for corolib".

  * in folder include/cppcoro: file_read_operation.hpp and file_write_operation.hpp: allow corolib to access functions try_start() (and some others).
  
  * in folder include/cppcoro/net: socket_accept_operation.hpp, socket_connect_operation.hpp, socket_disconnect_operation.hpp, socket_recv_from_operation.hpp, socket_recv_operation.hpp, socket_send_operation.hpp and socket_send_to_operation.hpp: allow corolib to access functions try_start() (and some others).

The directories examples-cc (cc = cppcoro) and examples-cl (cl = corolib) contain some client-server examples. 
Directory examples-cc uses cppcoro coroutines, while examples-cl uses corolib coroutines.
Files with the same name in both directory contain the same functionality,
but implemented with a different coroutine implementation.
