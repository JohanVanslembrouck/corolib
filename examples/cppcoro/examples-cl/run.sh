#!/bin/sh

set -x 

rm -f serveraddress.txt

./cl-cd_server2 &> cl-cd_server2.txt &
sleep 1
./cl-cd_client2
sleep 1

./cl-echo_server2 &> cl-echo_server2.txt &
sleep 1
./cl-echo_client2
sleep 1

./cl-udp_server2 &> cl-udp_server2.txt &
sleep 1
./cl-udp_client2
sleep 1

./cl-file_write_read2

sleep 1
cat cl-cd_server2.txt
cat cl-echo_server2.txt
cat cl-echo_server2.txt
cat cl-udp_server2.txt
