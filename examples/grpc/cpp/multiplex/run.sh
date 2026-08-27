#!/bin/sh
# Assumption: multiplex_server is already running

set -x

./multiplex_client
./multiplex_client2

./multiplex_coroutine_client2_ee
./multiplex_coroutine_client2_el
./multiplex_coroutine_client2_le
./multiplex_coroutine_client2_ll

./multiplex_coroutine_client3_ee
./multiplex_coroutine_client3_el
./multiplex_coroutine_client3_le
./multiplex_coroutine_client3_ll

./multiplex_coroutine_client3-when_all
./multiplex_coroutine_client3-when_any

# crashes
#./multiplex_coroutine_client3-all_ee
./multiplex_coroutine_client3-all_el
./multiplex_coroutine_client3-all_le
# craches
#./multiplex_coroutine_client3-all_ll

./multiplex_coroutine_client4

./multiplex_coroutine_client5
