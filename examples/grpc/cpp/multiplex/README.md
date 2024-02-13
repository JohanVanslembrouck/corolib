# gRPC C++ Multiplexing Example

This example shows how to use a single connection to send multiple concurrent
asynchronous requests to different services. It also demonstrates multiple
sharing the same server connection.

[C++ Quick Start]: https://grpc.io/docs/languages/cpp/quickstart

Launch the applications in the following order:
* multiplex_server(.exe)
* multiplex_client(.exe) or multiplex_client2(.exe) or multiplex_coroutine_client2(.exe) or multiplex_coroutine_client3(.exe) or multiplex_coroutine_client3-when_any(.exe)
