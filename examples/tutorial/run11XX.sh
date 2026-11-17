#!/bin/sh

set -x

./p1100-auto_reset_event-1
./p1101-auto_reset_event-1-when_all
./p1102-auto_reset_event-1-when_any
./p1103-auto_reset_event-2
./p1104-auto_reset_event-2-when_all
./p1105-auto_reset_event-2-when_any
./p1106-auto_reset_event-3
./p1107-auto_reset_event-3-when_all
./p1107-auto_reset_event-3-when_all_arr
./p1107-auto_reset_event-3-when_all_vec
./p1108-auto_reset_event-3-when_any
./p1108-auto_reset_event-3-when_any_arr
./p1108-auto_reset_event-3-when_any_vec

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
