
START /B cc-cd_server2.exe > cc-cd_server2.txt 2>&1
cc-cd_client2.exe

START /B cc-cd_server1.exe > cc-cd_server1.txt 2>&1
cc-cd_client1.exe

START /B cc-echo_server1.exe > cc-echo_server1.txt 2>&1
cc-echo_client1.exe

START /B cc-echo_server2.exe > cc-echo_server2.txt 2>&1
cc-echo_client2.exe

START /B cc-udp_server1.exe > cc-udp_server1.txt 2>&1
cc-udp_client1.exe

START /B cc-udp_server2.exe > cc-udp_server2.txt 2>&1
cc-udp_client2.exe

cc-file_write_read1.exe
cc-file_write_read2.exe

type cc-cd_server2.txt
type cc-cd_server1.txt
type cc-echo_server1.txt
type cc-echo_server2.txt
type cc-udp_server1.txt
type cc-udp_server2.txt
