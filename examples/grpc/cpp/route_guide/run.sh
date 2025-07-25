#!/bin/sh
# Assumption: route_guide_server or route_guide_callback_server is already running

set -x

./route_guide_client
./route_guide_callback_client
./route_guide_coroutine_client
./route_guide_coroutine_client2
