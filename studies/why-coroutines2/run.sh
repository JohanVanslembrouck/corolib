#!/bin/sh

set -x 

./p1000-sync
./p1001-sync

./p1010-sync-thread
./p1011-sync-thread

./p1020-async
./p1021-async
./p1025-async
./p1026-async

./p1030-async-thread
./p1031-async-thread

./p1040-coroutine
./p1041-coroutine
./p1042-coroutine

./p1060-corolib
./p1061-corolib
./p1062-corolib

./p1070-corolib-thread
./p1071-corolib-thread
./p1072-corolib-thread
