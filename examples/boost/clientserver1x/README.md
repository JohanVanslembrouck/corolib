# Three-tier client-server applications with eager/lazy start tasks and operations

This folder contains a subset of the applications in clientserver1.
It contains the same applications as clientserverlso.

The applications can be compiled in 4 ways:
* Eager start task (type async_task) + eager start operations: executable prefix = cs1xee-
* Eager start task (type async_task) + lazy start operations: executable prefix = cs1xel-
* Lazy start task (type async_ltask) + eager start operations: executable prefix = cs1xle-
* Lazy start task (type async_ltask) + lazy start operations: executable prefix : cs1xll-

This means that 4 client1 (or client1a) variants x 4 clientserver variants x 4 server variants = 64 combinations are possible!
