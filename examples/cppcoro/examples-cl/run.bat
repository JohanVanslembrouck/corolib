
cl-cd_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cl-cd_server2.exe > cl-cd_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cl-cd_client2.exe
timeout /t 1 /nobreak >nul
type cl-cd_server2.txt
timeout /t 1 /nobreak >nul

REM taskkill /im cl-cd_server2.exe /f /t

cl-echo_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cl-echo_server2.exe > cl-echo_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cl-echo_client2.exe
timeout /t 1 /nobreak >nul
type cl-echo_server2.txt
timeout /t 1 /nobreak >nul

REM taskkill /im cl-echo_server2.exe /f /t

REM cl-multiple_echo_server2_client2.exe
REM timeout /t 1 /nobreak >nul

REM START /B cl-multiple_echo_server2.exe > cl-multiple_echo_server2.txt 2>&1
REM timeout /t 1 /nobreak >nul
REM cl-multiple_echo_client2.exe
REM timeout /t 1 /nobreak >nul
REM type cl-multiple_echo_server2.txt
REM timeout /t 1 /nobreak >nul

cl-udp_server2_client2.exe
timeout /t 1 /nobreak >nul

START /B cl-udp_server2.exe > cl-udp_server2.txt 2>&1
timeout /t 1 /nobreak >nul
cl-udp_client2.exe
timeout /t 1 /nobreak >nul
type cl-udp_server2.txt
timeout /t 1 /nobreak >nul

cl-file_write_read2.exe
