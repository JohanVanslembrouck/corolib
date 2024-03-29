# Client-server application using Qt

This folder contains several Qt projects: tcpserver01.pro, tcpserver02.pro, tcpclient01.pro, tcpclient02.pro and tcpclient03.pro.
Build them using Qt Creator or qmake using qt5.pro in the corolib root diretory.

After building, launch the applications in the following order:

* tcpserver01.exe, tcpclient01.exe, or
* tcpserver02a.bat, tcpserver02b.bat, tcpclient02.exe, or
* tcpserver02a.bat, tcpserver02b.bat, tcpclient03.exe

The applications have companion configuration files (tcpserver01.cfg, tcpclient01.cfg, tcpserver02a.cfg, tcpserver02b.cfg, tcpclient02.cfg and tcpclient03.cfg) 
that allow selecting several run-time options.
The two .bat files start tcpserver02.exe with a different configuration. tcpclient02.exe and tcpclient03.exe expect two running tcpserver02 servers.
Copy the .cfg and the .bat files to the corresponding build folders (if different from this folder). 
On Windows, copy also Qt5Core.dll and Qt5Network.dll (or Qt5Cored.dll and Qt5Networkd.dll for the debug version) to the build folder.
Alternatively, place the original location of these files in the Path environment variable. 
For example, on my computer this is: C:\Qt\5.15.2\msvc2019_64\bin

The applications are usually built in separate directories. We assume 5 release build directories:
* the tcpserver01 build directory contains tcpserver01.exe and requires the following additional files: tcpserver01.cfg, Qt5Core.dll, Qt5Network.dll
* the tcpclient01 build directory contains tcpclient01.exe and requires the following additional files: tcpclient01.cfg, Qt5Core.dll, Qt5Network.dll
* the tcpserver02 build directory contains tcpserver02.exe and requires the following additional files: tcpserver02a.cfg, tcpserver02b.cfg, tcpserver02a.bat, tcpserver02b.bat, Qt5Core.dll, Qt5Network.dll
* the tcpclient02 build directory contains tcpclient02.exe and requires the following additional files: tcpclient02.cfg, Qt5Core.dll, Qt5Network.dll
* the tcpclient03 build directory contains tcpclient03.exe and requires the following additional files: tcpclient03.cfg, Qt5Core.dll, Qt5Network.dll

The major difference between tcplient01 and tcpclient02/tcpclient03 is that the latter have 2 TcpClient objects that communicate with the server, while the former has only 1 TcpClient object.
Because of this, the tcpclient01 application can handle the results of its sole client object in the application itself, in its start_reading() function.
The tcpclient02/tcpclient03 applications, on the other hand, have to make a stricter distinction and use the start_reading() function of its TcpClientCo/TcpClientCo1 objects. 
The tcpclient02/tcpclient03 applications also implement many more tests than tcpclient01.

The tcpclient03 application is mainly a copy-and-paste of the tcpclient02 application, but it uses two TcpClientCo1 objects instead of two TcpClientCo objects.
The TcpClientCo1 class reduces making connections between signals and slots to a minimum. TcpClientCo1 reuses an existing connection instead of always
making a connection between a signal and a slot and disconnecting it afterwards.

The tcpserver01 application does not use coroutines. Both client applications can be configured to use (or not use) coroutines, see variable useCoroutines in their .cfg files.

Each client application generates messages with lengths in the range [10, 10 + numberMessages[.
A client object (m_tcpClient in tcpclient01.h, m_tcpClient1 and m_tcpClient2 in tcpclient02.h and tcpclient03.h) 
then sends each message of a given length numberTransactions times to the server. 
The server just reflects the received message to the client.
The client co_awaits the reply for every sent message.
So, in essence, the client applications use a double loop. 

* Using coroutines, this double loop can be implemented as two nested for loops in a single coroutine, as can be seen in TcpClient02::measurementLoop20 till TcpClient02::measurementLoop40. 
(Same for the corresponding TcpClient03 member functions.)
* Without coroutines, the two counters have to be initialized, incremented and tested in several functions. This makes it more difficult to recognize the double loop pattern.

The coroutine approach leads to a more natural synchronous style of coding, yet the execution is asynchronous. 
This becomes even more clear in the coroutines TcpClient02::measurementLoop41 till TcpClient02::measurementLoop43, 
where the execution of the two invocations of TcpClient02::measurementLoop40 is interleaved. (Same for the correponding TcpClient03 member functions.)
