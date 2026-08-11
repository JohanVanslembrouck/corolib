# gRPC C++ Hello Streaming World Example

Original source code: https://groups.google.com/g/grpc-io/c/2wyoDZT5eao
Uses ../../protos/hellostreamingworld.proto

This folder contains examples of server-side streaming, where the server sends multiple HelloReply-s to the client.
The examples show how to use coroutines on the client side to deal with server-side streaming.

Launch the applications in the following order:
* multigreeter_server(.exe)
* multigreeter_client(.exe) or multigreeter_coroutine_client(.exe) or multigreeter_coroutine_client2(.exe) or any other client application
