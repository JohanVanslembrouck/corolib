# Three-tier client-server with eager/lazy start tasks and operations

This folder contains a subset of the applications in clientserver1.
It contains the same applications as clientserverlso.

The applications can be compiled in 4 ways:
* Eager start task (type async_task) + eager start operations: executable prefix = cs1xee-
* Eager start task (type async_task) + lazy start operations: executable prefix = cs1xel-
* Lazy start task (type async_ltask) + eager start operations: executable prefix = cs1xle-
* Lazy start task (type async_ltask) + lazy start operations: executable prefix : cs1xll-

This means that 4 client1 (or client1a) variants x 4 clientserver variants x 4 server variants = 64 combinations are possible!

After building, launch the applications in the following order:

* 'cs1xee-server' or 'cs1xel-server' or 'cs1xle-server' or 'cs1xll-server' and
* 'cs1xee-clientserver' or 'cs1xel-clientserver' or 'cs1xle-clientserver' or 'cs1xll-clientserver' and
* 'cs1xee-client1' or 'cs1xel-client1' or 'cs1xle-client1' or 'cs1xll-client1' (one or more instances) and/or 
* 'cs1xee-client1a' or 'cs1xel-client1a' or 'cs1xle-client1a' or 'cs1xll-client1a' (one or more instances) and/or 
* 'cs1xee-client3WA' or 'cs1xel-client3WA' or 'cs1xle-client3WA' or 'cs1xll-client3WA' (one or more instances) and/or 
* 'cs1xee-client3WAny' or 'cs1xel-client3WAny' or 'cs1xle-client3WAny' or 'cs1xll-client3WAny' (one or more instances)
