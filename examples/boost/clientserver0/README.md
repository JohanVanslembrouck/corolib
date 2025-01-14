# Two tier client-server application without coroutines

The applications in this directory do not use coroutines.
They have been based upon p0700c.cpp, p0710c.cpp and p0700s.cpp in studies/corolab.

In contrast to the applications in studies/corolab, the applications in this directory use as much as possible 
lambdas instead of callback functions.
The classes in the common directory that implement corouties also use lambdas,
which allow comparing the different lambda implementations.

The applications in this directory allow comparing this style of writing asynchronous applications
with the coroutine style used in the directories clientserver1 till clientserver4.

After building, launch the applications in the following order:

* 'cs0-server'
* 'cs0-client1' or 'cs0-client2' or 'cs0-client3'