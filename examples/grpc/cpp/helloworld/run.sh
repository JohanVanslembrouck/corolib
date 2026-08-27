#!/bin/sh
# Assumption: greeter_server (or greeter_async_server or greeter_callback_server) is already running

set -x

./greeter_client

./greeter_async_client
# The following program does not terminate
# ./greeter_async_client2

./greeter_callback_client

./greeter_cb_coroutine_client_xe
./greeter_cb_coroutine_client_xl

./greeter_cb_coroutine_client2_xe
./greeter_cb_coroutine_client2_xl

./greeter_cb_coroutine_client3_xe
./greeter_cb_coroutine_client3_xl

./greeter_coroutine_client_ee
./greeter_coroutine_client_el
# The following 2 programs block at run-time. See source code for explanation.
# greeter_coroutine_client_le
# greeter_coroutine_client_ll

./greeter_coroutine_client2_ee
./greeter_coroutine_client2_el
./greeter_coroutine_client2_le
./greeter_coroutine_client2_ll

./greeter_coroutine_client2a_ee
# crashes
#./greeter_coroutine_client2a_el
./greeter_coroutine_client2a_le
# crashes
#./greeter_coroutine_client2a_ll
