# gRPC Basics: C++ sample code

The files in this folder are the samples used in [gRPC Basics: C++][],
a detailed tutorial for using gRPC in C++.

[gRPC Basics: C++]:https://grpc.io/docs/languages/cpp/basics
Original source code: https://github.com/grpc/grpc/tree/master/examples/cpp/route_guide

Make sure that route_guide_db.json is in the same directory as the executables (or add the path to this file as command-line argument)

Launch the applications in the following order:
* route_guide_callback_server(.exe) or route_guide_server(.exe)
* route_guide_callback_client(.exe) or route_guide_client(.exe) or route_guide_coroutine_client(.exe) or route_guide_coroutine_client2(.exe)
