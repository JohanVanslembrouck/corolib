
START /B cl-cd_server2.exe > cl-cd_server2.txt 2>&1
cl-cd_client2.exe

REM taskkill /im cl-cd_server2.exe /f /t

START /B cl-echo_server2.exe > cl-echo_server2.txt 2>&1
cl-echo_client2.exe

REM taskkill /im cl-echo_server2.exe /f /t

START /B cl-udp_server2.exe > cl-udp_server2.txt 2>&1
cl-udp_client2.exe

cl-file_write_read2.exe

type cl-cd_server2.txt
type cl-echo_server2.txt
type cl-echo_server2.txt
type cl-udp_server2.txt
