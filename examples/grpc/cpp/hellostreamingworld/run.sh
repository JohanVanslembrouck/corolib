#!/bin/sh
# Assumption: multigreeter_server is already running

set -x

./multigreeter_client
./multigreeter_coroutine_client
./multigreeter_coroutine_client2
