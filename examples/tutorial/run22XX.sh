#!/bin/sh

set -x

./p2200-serial
./p2200-future
./p2200-future2

./p2200-async_operation
./p2202-async_operation-eventqueue
./p2204-async_operation-thread
./p2205-async_operation-thread-queue
./p2206-async_operation-immediate

# incomplete implementation:
#./p2200-async_operation_lso
./p2202-async_operation-eventqueue_lso
./p2204-async_operation-thread_lso
./p2205-async_operation-thread-queue_lso
./p2206-async_operation-immediate

./p2210-future
./p2210-future2
./p2210-no-coroutines

./p2210-async_operation
./p2212-async_operation-eventqueue
./p2214-async_operation-thread
./p2215-async_operation-thread-queue
./p2216-async_operation-immediate

# incomplete implementation:
#./p2210-async_operation_lso
./p2212-async_operation-eventqueue_lso
./p2214-async_operation-thread_lso
./p2215-async_operation-thread-queue_lso
./p2216-async_operation-immediate_lso

./p2220-no-coroutines
