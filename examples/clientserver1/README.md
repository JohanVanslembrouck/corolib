# Three-tier client-server application

'server' is a pure server application with 'clientserver' as its client.
'clientserver' is a combined client/server application: it a a client for 'server' 
and it acts as a server for any of the 3 pure client applications: 'client1', 'client3' and'client3WA'.

After building, launch the applications in the following order:

* 'server' and 'clientserver' (in any order)
* 'client1' (one or more instances) and/or 'client3' (one or more instances) and/or 'client3WA' (one or more instances)

For every incoming client request, 'clientserver' makes an interaction with 'server' that runs 
independently from the interaction with the client.
