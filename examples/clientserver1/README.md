# Three-tier client-server application

(For the relationship between executables and source files, the reader is referred to CMakeLists.txt in this directory.)

'cs1-server' is a pure server application with 'cs1-clientserver' as its client.
'cs1-clientserver' is a combined client/server application: it a a client for 'cs1-server' and it acts as a server for any of the 4 pure client applications: 'cs1-client1', 'cs1-client3', 'cs1-client3WA' and 'cs1-client3WAny'.

After building, launch the applications in the following order:

* 'cs1-server' and 'cs1-clientserver' (in any order)
* 'cs1-client1' (one or more instances) and/or 'cs1-client3' (one or more instances) and/or 'cs1-client3WA' (one or more instances) and/or 'cs1-client3WAny' (one or more instances)

For every incoming client request, 'cs1-clientserver' makes an interaction with 'cs1-server' that runs 
independently from the interaction with the client.
