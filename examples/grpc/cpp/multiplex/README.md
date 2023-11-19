# gRPC C++ Multiplexing Example

This example shows how to use a single connection to send multiple concurrent
asynchronous requests to different services. It also demonstrates multiple
sharing the same server connection.

[C++ Quick Start]: https://grpc.io/docs/languages/cpp/quickstart

Launch the applications in the following order:
* multiplex_server(.exe)
* multiplex_client(.exe) or multiplex_client2(.exe) or multiplex_coroutine_client2(.exe) or multiplex_coroutine_client3(.exe)

Notice that the execution of the coroutine variants is not completely reliable: 
they may finish correctly, hang at the t.wait(); statement in main() or even crash.
The reason is that some bool members in async_task and async_operation have to be turned into std::atomic\<bool> members when
used in multi-threaded applications (as is the case here), while a non-atomic version should be used (for performance reasons) 
in single-threaded applications. This is future work.
