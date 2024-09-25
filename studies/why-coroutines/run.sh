#!/bin/sh

echo Running ./p1000-sync-1rmi
./p1000-sync-1rmi
echo Running ./p1002-sync+thread-1rmi
./p1002-sync+thread-1rmi
echo Running ./p1004-sync+thread-1rmi
./p1004-sync+thread-1rmi
echo Running ./p1010-async-1rmi
./p1010-async-1rmi
echo Running ./p1020-coroutines-1rmi
./p1020-coroutines-1rmi
echo Running ./p1100-sync-callstack-1rmi
./p1100-sync-callstack-1rmi
echo Running ./p1110-async-callstack-1rmi
./p1110-async-callstack-1rmi
echo Running ./p1112-async-callstack-1rmi-cs
./p1112-async-callstack-1rmi-cs
echo Running ./p1120-coroutines-callstack-1rmi
./p1120-coroutines-callstack-1rmi
echo Running ./p1122-coroutines-callstack-1rmi
./p1122-coroutines-callstack-1rmi
echo Running ./p1124-coroutines-callstack-1rmi
./p1124-coroutines-callstack-1rmi
echo Running ./p1126-coroutines-callstack-1rmi
./p1126-coroutines-callstack-1rmi
echo Running ./p1130-coroutines-async-callstack-1rmi
./p1130-coroutines-async-callstack-1rmi
echo Running ./p1132-coroutines-async-callstack-1rmi-cs
./p1132-coroutines-async-callstack-1rmi-cs
echo Running ./p1200-sync-3rmis
./p1200-sync-3rmis
echo Running ./p1202-sync+thread-3rmis
./p1202-sync+thread-3rmis
echo Running ./p1210-async-3rmis
./p1210-async-3rmis
echo Running ./p1212-async-3rmis-local-event-loop
./p1212-async-3rmis-local-event-loop
echo Running ./p1220-coroutines-3rmis
./p1220-coroutines-3rmis
echo Running ./p1222-coroutines-3rmis-generichandler
./p1222-coroutines-3rmis-generichandler
echo Running ./p1500-sync-3-parallel-rmis
./p1500-sync-3-parallel-rmis
echo Running ./p1510-async-3-parallel-rmis
./p1510-async-3-parallel-rmis
echo Running ./p1520-coroutines-3-parallel-rmis
./p1520-coroutines-3-parallel-rmis
echo Running ./p1300-sync-nested-loop
./p1300-sync-nested-loop
echo Running ./p1310-async-nested-loop
./p1310-async-nested-loop
echo Running ./p1320-coroutines-nested-loop
./p1320-coroutines-nested-loop
echo Running ./p1400-sync-segmentation
./p1400-sync-segmentation
echo Running ./p1410-async-segmentation
./p1410-async-segmentation
echo Running ./p1420-coroutines-segmentation
./p1420-coroutines-segmentation
