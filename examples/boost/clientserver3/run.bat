@ECHO OFF

CALL :runserver cs3-server.exe
CALL :runserver cs3-server2.exe
CALL :runserver cs3-server3.exe
CALL :runserver cs3-server4.exe
CALL :runserver cs3-server5.exe
CALL :runserver cs3-server6.exe
CALL :runserver cs3-server7.exe
CALL :runserver cs3-server8.exe
CALL :runserver cs3-server8a.exe
CALL :runserver cs3-server9.exe
CALL :runserver cs3-server9a.exe
CALL :runserver cs3-server10.exe
CALL :runserver cs3-server10a.exe
CALL :runserver cs3-server11a.exe

GOTO :EOF

:runserver
START /B %1 > %1.txt 2>&1
cs3-client2.exe
echo "Killing %1 process"
taskkill /f /im %1 /t
timeout /t 2
type %1.txt
timeout /t 2
EXIT /B
