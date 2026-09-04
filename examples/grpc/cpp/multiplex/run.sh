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

./multiplex_coroutine_client3-when_all_ee
./multiplex_coroutine_client3-when_all_el
./multiplex_coroutine_client3-when_all_le
./multiplex_coroutine_client3-when_all_ll

./multiplex_coroutine_client3-when_any_le
./multiplex_coroutine_client3-when_any_el
./multiplex_coroutine_client3-when_any_le
./multiplex_coroutine_client3-when_any_ll

./multiplex_coroutine_client3-all_ee
./multiplex_coroutine_client3-all_el
./multiplex_coroutine_client3-all_le
./multiplex_coroutine_client3-all_ll

./multiplex_coroutine_client4_ee
./multiplex_coroutine_client4_el
./multiplex_coroutine_client4_le
./multiplex_coroutine_client4_ll

./multiplex_coroutine_client5_ee
./multiplex_coroutine_client5_el
./multiplex_coroutine_client5_le
./multiplex_coroutine_client5_ll
