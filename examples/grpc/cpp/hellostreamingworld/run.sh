#!/bin/sh
# Assumption: multigreeter_server is already running

set -x

./multigreeter_client

./multigreeter_coroutine_client_nc
./multigreeter_coroutine_client2_nc
./multigreeter_coroutine_client3_nc

./multigreeter_coroutine_client_ee
./multigreeter_coroutine_client_el
./multigreeter_coroutine_client_le
./multigreeter_coroutine_client_ll

./multigreeter_coroutine_client2_ee
./multigreeter_coroutine_client2_el
./multigreeter_coroutine_client2_le
./multigreeter_coroutine_client2_ll

./multigreeter_coroutine_client3_ee
./multigreeter_coroutine_client3_el
./multigreeter_coroutine_client3_le
./multigreeter_coroutine_client3_ll
