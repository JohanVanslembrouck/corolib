# Client-server application using Qt

This folder contains three Qt projects: tcpserver01.pro, tcpclient01.pro and tcpclient02.pro.
Build them using Qt Creator or qmake.

After building, launch the applications in the following order:

* 'tcpserver01.exe'
* 'tcpclient01.exe' or 'tcpclient02.exe'

The three applications have a companion configuration file (tcpserver01.cfg, tcpclient01.cfg and tcpclient02.cfg) that allow selecting several run-time options.
Copy the configuration files to the build folder (if different from this folder). Copy also Qt5Core.dll and Qt5Network.dll (or Qt5Cored.dll and Qt5Networkd.dll for the debug version) to the build folder.

The major difference between tcplient01 and tcpclient02 is that the latter has 2 TcpClient objects that communicate with the server, while the former has only 1 TcpClient object.
Because of this, the tcpclient01 application can handle the results of its sole client object in the application itself, in its start_reading() function.
The tcpclient02 application, on the other hand, has to make a stricter distinction and uses the start_reading() function of its TcpClient objects. The tcpclient02 application also implements many more tests than tcpclient01.

The tcpserver01 application does not use coroutines. Both client applications can be configured to use (or not use) coroutines, see variable useCoroutines in their .cfg files.
