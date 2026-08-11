#!/bin/sh
# Assumption: multigreeter_server is already running

set -x

./multigreeter_client

./multigreeter_coroutine_client_nc
./multigreeter_coroutine_client2_nc
./multigreeter_coroutine_client3_nc

./multigreeter_coroutine_client
./multigreeter_coroutine_client_lso
./multigreeter_coroutine_client2
./multigreeter_coroutine_client2_lso
./multigreeter_coroutine_client3
./multigreeter_coroutine_client3_lso
