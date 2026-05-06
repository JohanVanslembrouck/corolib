# Three-tier client-server applications with lazy start operations (lso)

This folder contains a subset of the applications in clientserver1.

Instead of using eager start operations (defined in sibling folder common),
it uses lazy start operations defined in sibling folder commonlso.

The applications can be compiled in 2 ways:
* Eager start task (type async_task) + lazy start operations: executable prefix = cs1elso-
* Lazy start task (type async_ltask) + lazy start operations: executable prefix = cs1llso-

This means that 2 client1 (or client1a) variants x 2 clientserver variants x 2 server variants = 8 combinations are possible!
