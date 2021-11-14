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
The tcpclient02 application, on the other hand, has to make a stricter distinction and uses the start_reading() function of its TcpClientCo(routine) objects. The tcpclient02 application also implements many more tests than tcpclient01.

The tcpserver01 application does not use coroutines. Both client applications can be configured to use (or not use) coroutines, see variable useCoroutines in their .cfg files.

Each client application generates messages of different lengths in the range [10, 10 + numberMessages[.
A client object (m_tcpClient in tcpclient01.h, m_tcpClient and m_tcpClient in tcpclient02.h) 
sends each message of a given length numberTransactions times to the server. The application then calculates the time for one transaction.
(A transaction consists of sending a single message to the server application and the server sending back the message to the client.)
So, in essence, the client applications use a double loop. Using coroutines, this double loop can be implemented as two nested for loops in a single coroutine, as can be seen in TcpClient02::measurementLoop20 till TcpClient02::measurementLoop40.
Without coroutines, counting the number of transactions and the messages of various sizes has to be implemented with two counters initialized, incremented and tested in several functions. This makes it more difficult to recognize the double loop pattern.
The coroutine approach leads to a more natural synchronous style of writing, but the execution is asynchronous. 
This becomes even more clear in the coroutines TcpClient02::measurementLoop41 till TcpClient02::measurementLoop43, where the execution of the two invocations of TcpClient02::measurementLoop40 is interleaved.
