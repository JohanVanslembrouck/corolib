#!/bin/sh

set -x

rm -f serveraddress.txt

./cc-cd_server2 &> cc-cd_server2.txt &
sleep 1
./cc-cd_client2
sleep 1

./cc-cd_server1 &> cc-cd_server1.txt &
sleep 1
./cc-cd_client1
sleep 1

./cc-echo_server1 &> cc-echo_server1.txt &
sleep 1
./cc-echo_client1
sleep 1

./cc-echo_server2 &> cc-echo_server2.txt &
sleep 1
./cc-echo_client2
sleep 1

./cc-udp_server1 &> cc-udp_server1.txt &
sleep 1
./cc-udp_client1
sleep 1

./cc-udp_server2 &> cc-udp_server2.txt &
sleep 1
./cc-udp_client2
sleep 1

./cc-file_write_read1
./cc-file_write_read2

sleep 1
cat cc-cd_server2.txt
cat cc-cd_server1.txt
cat cc-echo_server1.txt
cat cc-echo_server2.txt
cat cc-udp_server1.txt
cat cc-udp_server2.txt
