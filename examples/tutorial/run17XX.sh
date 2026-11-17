#!/bin/sh

set -x

./p1700-async_operation
./p1700a-async_operation
./p1701-async_operation-exception
./p1701x-async_operation-exception
./p1706-async_operation-immediate
./p1706a-async_operation-immediate

./p1730-async_operation
./p1730a-async_operation
./p1731-async_operation-exception
./p1731x-async_operation-exception
./p1732-async_operation-eventqueue
./p1734-async_operation-thread
./p1735-async_operation-thread-queue
./p1736-async_operation-immediate
