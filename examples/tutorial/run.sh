#!/bin/sh

echo Running ./p1000
./p1000
echo Running ./p1001-recursive
./p1001-recursive
#echo Running ./p1002-no-coroutines
#./p1002-no-coroutines
#echo Running ./p1010-thread
#./p1010-thread
#echo Running ./p1011-thread-recursive
#./p1011-thread-recursive
echo Running ./p1050-resume_same_thread
./p1050-resume_same_thread
echo Running ./p1052-resume_same_thread
./p1052-resume_same_thread
echo Running ./p1054-resume_same_thread
./p1054-resume_same_thread
echo Running ./p1060-resume_new_thread
./p1060-resume_new_thread
echo Running ./p1062-resume_new_thread
./p1062-resume_new_thread
echo Running ./p1064-resume_new_thread
./p1064-resume_new_thread

echo Running ./p1100-auto_reset_event-1
./p1100-auto_reset_event-1
echo Running ./p1101-auto_reset_event-1-when_all
./p1101-auto_reset_event-1-when_all
echo Running ./p1102-auto_reset_event-1-when_any
./p1102-auto_reset_event-1-when_any
echo Running ./p1103-auto_reset_event-2
./p1103-auto_reset_event-2
echo Running ./p1104-auto_reset_event-2-when_all
./p1104-auto_reset_event-2-when_all
echo Running ./p1105-auto_reset_event-2-when_any
./p1105-auto_reset_event-2-when_any
echo Running ./p1106-auto_reset_event-3
./p1106-auto_reset_event-3
echo Running ./p1107-auto_reset_event-3-when_all
./p1107-auto_reset_event-3-when_all
echo Running ./p1108-auto_reset_event-3-when_any
./p1108-auto_reset_event-3-when_any
echo Running ./p1110-auto_reset_event-thread-1
./p1110-auto_reset_event-thread-1
echo Running ./p1111-auto_reset_event-thread-1-when_all
./p1111-auto_reset_event-thread-1-when_all
echo Running ./p1112-auto_reset_event-thread-1-when_any
./p1112-auto_reset_event-thread-1-when_any
echo Running ./p1113-auto_reset_event-thread-2
./p1113-auto_reset_event-thread-2
echo Running ./p1114-auto_reset_event-thread-2-when_all
./p1114-auto_reset_event-thread-2-when_all
echo Running ./p1115-auto_reset_event-thread-2-when_any
./p1115-auto_reset_event-thread-2-when_any
echo Running ./p1116-auto_reset_event-thread-3
./p1116-auto_reset_event-thread-3
echo Running ./p1117-auto_reset_event-thread-3-when_all
./p1117-auto_reset_event-thread-3-when_all
echo Running ./p1118-auto_reset_event-thread-3-when_any
./p1118-auto_reset_event-thread-3-when_any
echo Running ./p1120-auto_reset_event-oneway_task
./p1120-auto_reset_event-oneway_task

echo Running ./p1200-mini0
./p1200-mini0
echo Running ./p1210-mini1-thread
./p1210-mini1-thread
echo Running ./p1220-mini1-oneway_task-thread
./p1220-mini1-oneway_task-thread
# ./p1300-future

echo Running ./p1400-async_operation
./p1400-async_operation
echo Running ./p1402-async_operation-eventqueue
./p1402-async_operation-eventqueue
echo Running ./p1404-async_operation-thread
./p1404-async_operation-thread
echo Running ./p1405-async_operation-thread-queue
./p1405-async_operation-thread-queue
echo Running ./p1406-async_operation-immediate
./p1406-async_operation-immediate
echo Running ./p1410-async_operation
./p1410-async_operation
echo Running ./p1411-async_operation-exception
./p1411-async_operation-exception
echo Running ./p1412-async_operation-eventqueue
./p1412-async_operation-eventqueue
echo Running ./p1414-async_operation-thread
./p1414-async_operation-thread
echo Running ./p1415-async_operation-thread-queue
./p1415-async_operation-thread-queue
echo Running ./p1416-async_operation-immediate
./p1416-async_operation-immediate
echo Running ./p1420-async_operation
./p1420-async_operation
echo Running ./p1421-async_operation-exception
./p1421-async_operation-exception
echo Running ./p1422-async_operation-eventqueue
./p1422-async_operation-eventqueue
echo Running ./p1424-async_operation-thread
./p1424-async_operation-thread
echo Running ./p1425-async_operation-thread-queue
./p1425-async_operation-thread-queue
echo Running ./p1426-async_operation-immediate
./p1426-async_operation-immediate
echo Running ./p1428-async_operation-evtq-imm
./p1428-async_operation-evtq-imm
echo Running ./p1429-async_operation-thread-imm
./p1429-async_operation-thread-imm
echo Running ./p1430-async_operation
./p1430-async_operation
echo Running ./p1430a-async_operation
./p1430a-async_operation
echo Running ./p1432-async_operation-eventqueue
./p1432-async_operation-eventqueue
echo Running ./p1434-async_operation-thread
./p1434-async_operation-thread
echo Running ./p1435-async_operation-thread-queue
./p1435-async_operation-thread-queue
echo Running ./p1436-async_operation-immediate
./p1436-async_operation-immediate
echo Running ./p1438-async_operation-evtq-imm
./p1438-async_operation-evtq-imm
echo Running ./p1439-async_operation-thread-imm
./p1439-async_operation-thread-imm
echo Running ./p1440-async_operation
./p1440-async_operation
echo Running ./p1442-async_operation-eventqueue
./p1442-async_operation-eventqueue
echo Running ./p1444-async_operation-thread
./p1444-async_operation-thread
echo Running ./p1455-async_operation-thread-queue
./p1455-async_operation-thread-queue
echo Running ./p1446-async_operation-immediate
./p1446-async_operation-immediate
echo Running ./p1448-async_operation-evtq-imm
./p1448-async_operation-evtq-imm
echo Running ./p1449-async_operation-thread-imm
./p1449-async_operation-thread-imm
echo Running ./p1450-async_operation
./p1450-async_operation
echo Running ./p1452-async_operation-eventqueue
./p1452-async_operation-eventqueue
echo Running ./p1454-async_operation-thread
./p1454-async_operation-thread
echo Running ./p1455-async_operation-thread-queue
./p1455-async_operation-thread-queue
echo Running ./p1456-async_operation-immediate
./p1456-async_operation-immediate
echo Running ./p1458-async_operation-evtq-imm
./p1458-async_operation-evtq-imm
echo Running ./p1459-async_operation-thread-imm
./p1459-async_operation-thread-imm
echo Running ./p1460-async_operation
./p1460-async_operation
echo Running ./p1462-async_operation-eventqueue
./p1462-async_operation-eventqueue
echo Running ./p1464-async_operation-thread
./p1464-async_operation-thread
echo Running ./p1465-async_operation-thread-queue
./p1465-async_operation-thread-queue
echo Running ./p1466-async_operation-immediate
./p1466-async_operation-immediate
echo Running ./p1468-async_operation-evtq-imm
./p1468-async_operation-evtq-imm
echo Running ./p1469-async_operation-thread-imm
./p1469-async_operation-thread-imm
echo Running ./p1472-async_operation-eventqueue
./p1472-async_operation-eventqueue
#echo Running ./p1474-async_operation-thread
#./p1474-async_operation-thread
# echo Running ./p1475-async_operation-thread-queue
# ./p1475-async_operation-thread-queue
# echo Running ./p1476-async_operation-immediate
# ./p1476-async_operation-immediate
echo Running ./p1482-async_operation-eventqueue
./p1482-async_operation-eventqueue
echo Running ./p1484-async_operation-thread
./p1484-async_operation-thread
echo Running ./p1485-async_operation-thread-queue
./p1485-async_operation-thread-queue

echo Running ./p1500-async_operation
./p1500-async_operation
echo Running ./p1600-async_operation
./p1600-async_operation
echo Running ./p1700-async_operation
./p1700-async_operation
echo Running ./p1700a-async_operation
./p1700a-async_operation
echo Running ./p1730-async_operation
./p1730-async_operation
echo Running ./p1730a-async_operation
./p1730a-async_operation
echo Running ./p1800-async_operation
./p1800-async_operation
echo Running ./p1802-async_operation-eventqueue
./p1802-async_operation-eventqueue
echo Running ./p1804-async_operation-thread
./p1804-async_operation-thread
echo Running ./p1805-async_operation-thread-queue
./p1805-async_operation-thread-queue
echo Running ./p1810-async_operation
./p1810-async_operation
echo Running ./p1812-async_operation-eventqueue
./p1812-async_operation-eventqueue
echo Running ./p1814-async_operation-thread
./p1814-async_operation-thread
echo Running ./p1815-async_operation-thread-queue
./p1815-async_operation-thread-queue
echo Running ./p1820-async_operation
./p1820-async_operation
echo Running ./p1822-async_operation-eventqueue
./p1822-async_operation-eventqueue
echo Running ./p1824-async_operation-thread
./p1824-async_operation-thread
echo Running ./p1825-async_operation-thread-queue
./p1825-async_operation-thread-queue
echo Running ./p1830-async_operation
./p1830-async_operation
echo Running ./p1832-async_operation-eventqueue
./p1832-async_operation-eventqueue
echo Running ./p1834-async_operation-thread
./p1834-async_operation-thread
echo Running ./p1835-async_operation-thread-queue
./p1835-async_operation-thread-queue
echo Running ./p1840-async_operation
./p1840-async_operation
echo Running ./p1842-async_operation-eventqueue
./p1842-async_operation-eventqueue
echo Running ./p1844-async_operation-thread
./p1844-async_operation-thread
echo Running ./p1845-async_operation-thread-queue
./p1845-async_operation-thread-queue
# echo Running ./p1900-resume_in_middle
# ./p1900-resume_in_middle
echo Running ./p1910-async_queue
./p1910-async_queue
echo Running ./p1912-async_queue-fibonacci
./p1912-async_queue-fibonacci

echo Running ./p2000-async_operation
./p2000-async_operation
echo Running ./p2002-async_operation-eventqueue
./p2002-async_operation-eventqueue
echo Running ./p2004-async_operation-thread
./p2004-async_operation-thread
echo Running ./p2005-async_operation-thread-queue
./p2005-async_operation-thread-queue
echo Running ./p2010-async_operation
./p2010-async_operation
echo Running ./p2012-async_operation-eventqueue
./p2012-async_operation-eventqueue
echo Running ./p2014-async_operation-thread
./p2014-async_operation-thread
echo Running ./p2015-async_operation-thread-queue
./p2015-async_operation-thread-queue
echo Running ./p2100-async_operation
./p2100-async_operation
echo Running ./p2102-async_operation-eventqueue
./p2102-async_operation-eventqueue
echo Running ./p2110-async_operation
./p2110-async_operation
echo Running ./p2112-async_operation-eventqueue
./p2112-async_operation-eventqueue
