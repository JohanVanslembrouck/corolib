#!/bin/sh

set -x

./p1800-async_operation
./p1802-async_operation-eventqueue
./p1804-async_operation-thread
./p1805-async_operation-thread-queue

./p1810-async_operation
./p1812-async_operation-eventqueue
./p1814-async_operation-thread
./p1815-async_operation-thread-queue

./p1820-async_operation
./p1822-async_operation-eventqueue
./p1824-async_operation-thread
./p1825-async_operation-thread-queue

./p1830-async_operation
./p1832-async_operation-eventqueue
./p1834-async_operation-thread
./p1835-async_operation-thread-queue

./p1840-async_operation
./p1842-async_operation-eventqueue
./p1844-async_operation-thread
./p1845-async_operation-thread-queue
