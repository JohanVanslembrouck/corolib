#!/bin/sh

set -x

./p1400-async_operation
./p1401-async_operation-exception
./p1401x-async_operation-exception
./p1402-async_operation-eventqueue
./p1404-async_operation-thread
./p1405-async_operation-thread-queue
./p1406-async_operation-immediate

./p1410-async_operation
./p1411-async_operation-exception
./p1411x-async_operation-exception
./p1412-async_operation-eventqueue
./p1414-async_operation-thread
./p1415-async_operation-thread-queue
./p1416-async_operation-immediate

./p1420-async_operation
./p1421-async_operation-exception
./p1421x-async_operation-exception
./p1422-async_operation-eventqueue
./p1424-async_operation-thread
./p1425-async_operation-thread-queue
./p1426-async_operation-immediate
./p1428-async_operation-evtq-imm
./p1429-async_operation-thread-imm

./p1430-async_operation
./p1430a-async_operation
./p1431-async_operation-exception
./p1431x-async_operation-exception
./p1431a-async_operation-exception
./p1431ax-async_operation-exception
./p1432-async_operation-eventqueue
./p1434-async_operation-thread
./p1435-async_operation-thread-queue
./p1436-async_operation-immediate
./p1438-async_operation-evtq-imm
./p1439-async_operation-thread-imm

./p1440-async_operation
./p1442-async_operation-eventqueue
./p1444-async_operation-thread
./p1455-async_operation-thread-queue
./p1446-async_operation-immediate
./p1448-async_operation-evtq-imm
./p1449-async_operation-thread-imm

./p1450-async_operation
./p1452-async_operation-eventqueue
./p1454-async_operation-thread
./p1455-async_operation-thread-queue
./p1456-async_operation-immediate
./p1458-async_operation-evtq-imm
./p1459-async_operation-thread-imm

./p1460-async_operation
./p1462-async_operation-eventqueue
./p1464-async_operation-thread
./p1465-async_operation-thread-queue
./p1466-async_operation-immediate
./p1468-async_operation-evtq-imm
./p1469-async_operation-thread-imm

./p1472-async_operation-eventqueue
./p1474-async_operation-thread
./p1475-async_operation-thread-queue
# does not terminate:
#./p1476-async_operation-immediate
