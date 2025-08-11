#!/bin/sh

set -x 

./p1000-sync-1rmi
./p1002-sync+thread-1rmi
./p1004-sync+thread-1rmi
./p1010-async-1rmi
./p1015-async+thread-1rmi
./p1020-coroutines-1rmi

./p1050-sync-1rmi
./p1060-async-1rmi
./p1070-coroutines-1rmi

./p1100-sync-callstack-1rmi
./p1110-async-callstack-1rmi
./p1112-async-callstack-1rmi-cs
./p1115-async-callstack+thread-1rmi
./p1120-coroutines-callstack-1rmi
./p1122-coroutines-callstack-1rmi
./p1124-coroutines-callstack-1rmi
./p1126-coroutines-callstack-1rmi
./p1130-coroutines-async-callstack-1rmi
./p1132-coroutines-async-callstack-1rmi-cs

./p1150-sync-callstack-1rmi
./p1160-async-callstack-1rmi
./p1170-coroutines-callstack-1rmi

./p1200-sync-3rmis
./p1202-sync+thread-3rmis
./p1210-async-3rmis
./p1212-async-3rmis-local-event-loop
./p1215-async+thread-3rmis
./p1220-coroutines-3rmis
./p1222-coroutines-3rmis-generichandler

./p1500-sync-3-parallel-rmis
./p1510-async-3-parallel-rmis
./p1515-async+thread-3-parallel-rmis
./p1520-coroutines-3-parallel-rmis

./p1300-sync-nested-loop
./p1310-async-nested-loop
./p1315-async+thread-nested-loop
./p1320-coroutines-nested-loop

./p1350-sync-nested-loop
./p1360-async-nested-loop
./p1370-coroutines-nested-loop

./p1400-sync-segmentation
./p1410-async-segmentation
./p1420-coroutines-segmentation

./p2000-sync-3rmis
./p2010-async-3rmis
./p2011-async-3rmis
./p2012-async-3rmis
./p2013-async-3rmis
./p2015-async+thread-3rmis
./p2016-async+thread-3rmis
./p2020-coroutines-3rmis
./p2021-coroutines-3rmis
./p2022-coroutines-3rmis
./p2023-coroutines-3rmis
