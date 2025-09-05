#!/bin/sh

set -x

./p1000
./p1001-recursive
#./p1002-no-coroutines
#./p1010-thread
#./p1011-thread-recursive
./p1050-resume_same_thread
./p1052-resume_same_thread
./p1054-resume_same_thread
./p1060-resume_new_thread
./p1062-resume_new_thread
./p1064-resume_new_thread

./p1100-auto_reset_event-1
./p1101-auto_reset_event-1-when_all
./p1102-auto_reset_event-1-when_any
./p1103-auto_reset_event-2
./p1104-auto_reset_event-2-when_all
./p1105-auto_reset_event-2-when_any
./p1106-auto_reset_event-3
./p1107-auto_reset_event-3-when_all
./p1108-auto_reset_event-3-when_any
./p1110-auto_reset_event-thread-1
./p1111-auto_reset_event-thread-1-when_all
./p1112-auto_reset_event-thread-1-when_any
./p1113-auto_reset_event-thread-2
./p1114-auto_reset_event-thread-2-when_all
./p1115-auto_reset_event-thread-2-when_any
./p1116-auto_reset_event-thread-3
./p1117-auto_reset_event-thread-3-when_all
./p1118-auto_reset_event-thread-3-when_any
./p1120-auto_reset_event-oneway_task

./p1200-mini0
./p1210-mini1-thread
./p1220-mini1-oneway_task-thread
# ./p1300-future

./p1400-async_operation
./p1402-async_operation-eventqueue
./p1404-async_operation-thread
./p1405-async_operation-thread-queue
./p1406-async_operation-immediate
./p1410-async_operation
./p1411-async_operation-exception
./p1412-async_operation-eventqueue
./p1414-async_operation-thread
./p1415-async_operation-thread-queue
./p1416-async_operation-immediate
./p1420-async_operation
./p1421-async_operation-exception
./p1422-async_operation-eventqueue
./p1424-async_operation-thread
./p1425-async_operation-thread-queue
./p1426-async_operation-immediate
./p1428-async_operation-evtq-imm
./p1429-async_operation-thread-imm
./p1430-async_operation
./p1430a-async_operation
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
#./p1474-async_operation-thread
# ./p1475-async_operation-thread-queue
# ./p1476-async_operation-immediate
./p1482-async_operation-eventqueue
./p1484-async_operation-thread
./p1485-async_operation-thread-queue

./p1500-async_operation
./p1600-async_operation
./p1610-async_operation

./p1700-async_operation
./p1700a-async_operation
./p1706-async_operation-immediate
./p1706a-async_operation-immediate
./p1730-async_operation
./p1730a-async_operation
./p1734-async_operation-thread
./p1736-async_operation-immediate

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

# ./p1900-resume_in_middle
./p1910-async_queue
./p1912-async_queue-fibonacci
./p1920-async_queue-async_file
./p1922-async_queue_eq-async_file

./p2000-async_operation
./p2002-async_operation-eventqueue
./p2004-async_operation-thread
./p2005-async_operation-thread-queue
./p2010-async_operation
./p2012-async_operation-eventqueue
./p2014-async_operation-thread
./p2015-async_operation-thread-queue
./p2100-async_operation
./p2102-async_operation-eventqueue
./p2110-async_operation
./p2112-async_operation-eventqueue
