#!/bin/sh
# Assumption: greeter_server (or greeter_async_server or greeter_callback_server) is already running

set -x

./greeter_client

./greeter_async_client
# The following program does not terminate
# ./greeter_async_client2

./greeter_callback_client

./greeter_cb_coroutine_client
./greeter_cb_coroutine_client2
./greeter_cb_coroutine_client3

./greeter_coroutine_client
./greeter_coroutine_client2
./greeter_coroutine_client2a
