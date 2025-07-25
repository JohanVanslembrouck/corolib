#!/bin/sh
# Assumption: multiplex_server is already running

set -x

./multiplex_client
./multiplex_client2

./multiplex_coroutine_client2

./multiplex_coroutine_client3
./multiplex_coroutine_client3-when_all
./multiplex_coroutine_client3-when_any
./multiplex_coroutine_client3-all

./multiplex_coroutine_client4

./multiplex_coroutine_client5
