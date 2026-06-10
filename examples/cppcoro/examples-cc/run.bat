
cc-cd_server1_client1.exe
timeout /t 1 /nobreak >nul

START /B cc-cd_server1.exe > cc-cd_server1.txt 2>&1
timeout /t 1 /nobreak >nul
cc-cd_client1.exe
timeout /t 1 /nobreak >nul
type cc-cd_server1.txt
timeout /t 1 /nobreak >nul

cc-cd_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cc-cd_server2.exe > cc-cd_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cc-cd_client2.exe
timeout /t 1 /nobreak >nul
type cc-cd_server2.txt
timeout /t 1 /nobreak >nul

cc-echo_server1_client1.exe
timeout /t 1 /nobreak >nul

START /B cc-echo_server1.exe > cc-echo_server1.txt 2>&1
timeout /t 1 /nobreak >nul
cc-echo_client1.exe
timeout /t 1 /nobreak >nul
type cc-echo_server1.txt
timeout /t 1 /nobreak >nul

cc-echo_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cc-echo_server2.exe > cc-echo_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cc-echo_client2.exe
timeout /t 1 /nobreak >nul
type cc-echo_server2.txt
timeout /t 1 /nobreak >nul

cc-udp_server1_client1.exe
timeout /t 1 /nobreak >nul

cc-multiple_echo_server1_client1.exe
timeout /t 1 /nobreak >nul

START /B cc-multiple_echo_server1.exe > cc-multiple_echo_server1.txt 2>&1
timeout /t 1 /nobreak >nul
cc-multiple_echo_client1.exe
timeout /t 1 /nobreak >nul
type cc-multiple_echo_server1.txt
timeout /t 1 /nobreak >nul

cc-multiple_echo_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cc-multiple_echo_server2.exe > cc-multiple_echo_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cc-multiple_echo_client2.exe
timeout /t 1 /nobreak >nul
type cc-multiple_echo_server2.txt
timeout /t 1 /nobreak >nul

START /B cc-udp_server1.exe > cc-udp_server1.txt 2>&1
timeout /t 1 /nobreak >nul
cc-udp_client1.exe
timeout /t 1 /nobreak >nul
type cc-udp_server1.txt
timeout /t 1 /nobreak >nul

cc-udp_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cc-udp_server2.exe > cc-udp_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cc-udp_client2.exe
timeout /t 1 /nobreak >nul
type cc-udp_server2.txt
timeout /t 1 /nobreak >nul

cc-file_write_read1.exe
timeout /t 1 /nobreak >nul
cc-file_write_read2.exe
