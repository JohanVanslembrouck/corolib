#!/bin/bash

set -x 

./cl-cd_server2_client2
sleep 1

./cl-cd_server2 &> cl-cd_server2.txt &
sleep 1
./cl-cd_client2
sleep 1

cat cl-cd_server2.txt
sleep 1

if false; then
    /cl-echo_server2_client2
    sleep 1

    ./cl-echo_server2 &> cl-echo_server2.txt &
    sleep 1
    /cl-echo_client2
    sleep 1
fi

cat cl-echo_server2.txt
sleep 1

./cl-udp_server2_client2
sleep 1

./cl-udp_server2 &> cl-udp_server2.txt &
sleep 1
./cl-udp_client2
sleep 1

cat cl-udp_server2.txt
sleep 1

./cl-file_write_read2
