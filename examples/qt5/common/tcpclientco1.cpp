/**
 * @file tcpclientco1.cpp
 * @brief Implementation of a TCP client class.
 * Uses coroutines. Variant of tcpclienco.cpp.
 * See README.md file for further explanation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclientco1.h"

/**
 * @brief TcpClientCo::TcpClientCo
 * @param name
 * @param autoConnect
 * @param waitForConnectionTimeout
 * @param reconnectTimeout
 * @param reconnectTimeoutAfterDisconnect
 */
TcpClientCo1::TcpClientCo1(qint32 selectImplementation,
                          const QString& name,
                          bool autoConnect,
                          qint32 waitForConnectionTimeout,
                          qint32 reconnectTimeout,
                          qint32 reconnectTimeoutAfterDisconnect)
    : m_selectImplementation(selectImplementation)
    , m_timer(this)
    , m_receiveTimer(this)
    , m_autoConnect(autoConnect)
    , m_name(name)
    , m_waitForConnectionTimeout(waitForConnectionTimeout)
    , m_reconnectTimeout(reconnectTimeout)
    , m_reconnectTimeoutAfterDisconnect(reconnectTimeoutAfterDisconnect)
{
    qInfo() << Q_FUNC_INFO << m_name << ", autoConnect = " << autoConnect;
}

/**
 * @brief TcpClientCo1::configure
 */
void TcpClientCo1::configure()
{
    qInfo() << Q_FUNC_INFO << m_name;

    // Client starts timer to reconnect to server if the connection was lost.
    m_timer.setSingleShot(true);
    m_receiveTimer.setSingleShot(true);

    connect(&m_timer,        &QTimer::timeout, this, &TcpClientCo1::connectToServerTimed);
    connect(&m_receiveTimer, &QTimer::timeout, this, &TcpClientCo1::receiveTimed);
}

/**
 * @brief TcpClientCo1::connectToServerTimed
 */
void TcpClientCo1::connectToServerTimed()
{
    qInfo() << Q_FUNC_INFO << m_name;

    connectToServer(m_serverIPaddress, m_serverPort);
}

/**
 * @brief TcpClientCo1::receiveTimed
 */
void TcpClientCo1::receiveTimed()
{
    qInfo() << Q_FUNC_INFO << m_name;

    QByteArray empty;
    readyReadTcpCo(empty);
}

/**
 * @brief TcpClientCo1::enableKeepAlive
 * @param socket
 */
void TcpClientCo1::enableKeepAlive(QTcpSocket *socket)
{
    // https://doc.qt.io/qt-5/qabstractsocket.html#setSocketOption
    // Note: On Windows Runtime, QAbstractSocket::KeepAliveOption must be set before the socket is connected.

    if (socket)
    {
        // https://doc.qt.io/qt-5/qabstractsocket.html#SocketOption-enum
        socket->setSocketOption(QAbstractSocket::KeepAliveOption, true);
    }
}

/**
 * @brief TcpClientCo1::connectToServer
 * @param serverIPaddress
 * @param serverPort
 * @return
 */
bool TcpClientCo1::connectToServer(QString& serverIPaddress, quint16 serverPort)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "serverIPaddress = " << serverIPaddress << ", serverPort = " << serverPort;

    bool retVal = false;

    m_serverIPaddress = serverIPaddress;
    m_serverPort = serverPort;

    QTcpSocket* socket = new QTcpSocket(this);
    if (socket)
    {
        socket->connectToHost(serverIPaddress, serverPort);
        qInfo() << "Start waiting";

        if (socket->waitForConnected(m_waitForConnectionTimeout))   // default value = 30000 msec
        {
            qDebug() << Q_FUNC_INFO << "Connected to server:" << serverIPaddress << ":" << serverPort;

            enableKeepAlive(socket);

            ConnectionInfo* connectionInfo = new ConnectionInfo;
            if (connectionInfo)
            {
                connectionInfo->m_socket = socket;
                connectionInfo->m_connection_ReadyRead    = connect(socket, &QTcpSocket::readyRead,    this, &TcpClientCo1::readyReadTcp);
                connectionInfo->m_connection_disconnected = connect(socket, &QTcpSocket::disconnected, this, &TcpClientCo1::disconnectedServer);
                connectionInfo->m_connection_stateChanged = connect(socket, &QTcpSocket::stateChanged, this, &TcpClientCo1::stateChanged);
                m_connectionInfoList.append(connectionInfo);

                // Begin extra code compared with tcpclientco.cpp
                m_connection_read = connect(this, &TcpClientCo1::responseReceivedSig,
                    [this](QByteArray msg)
                    {
                        int idx = m_index_read;

                        print(PRI2, "%p: TcpClientCo1::handle_read() lambda: idx = %d\n", this, idx);

                        async_operation_base* om_async_operation = get_async_operation(idx);
                        async_operation<QByteArray>* om_async_operation_t =
                            static_cast<async_operation<QByteArray>*>(om_async_operation);

                        if (om_async_operation_t)
                        {
                            om_async_operation_t->set_result(msg);
                            om_async_operation_t->completed();
                        }
                        else
                        {
                            // This can occur when the async_operation_base has gone out of scope.
                            print(PRI2, "%p: TcpClientCo::handle_read(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
                        }
                    }
                );
                
                m_connection_connect = connect(this, &TcpClientCo1::connectedSig,
                    [this]()
                    {
                        int idx = m_index_connect;

                        print(PRI2, "%p: TcpClientCo1::handle_connect() lambda: idx = %d\n", this, idx);

                        async_operation_base* om_async_operation = get_async_operation(idx);
                        async_operation<void>* om_async_operation_t =
                            static_cast<async_operation<void>*>(om_async_operation);

                        if (om_async_operation_t)
                        {
                            om_async_operation_t->completed();
                        }
                        else
                        {
                            // This can occur when the async_operation_base has gone out of scope.
                            print(PRI2, "%p: TcpClientCo1::handle_connect(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
                        }
                    }
                );
#if 0
                m_connection_timer = connect(&tmr, &QTimer::timeout,
                    [this]()
                    {
                        int idx = m_index_timer;
                        
                        print(PRI2, "%p: TcpClientCo1::handle_timer() lambda: idx = %d\n", this, idx);

                        async_operation_base* om_async_operation = m_async_operations[idx];
                        async_operation<void>* om_async_operation_t =
                            static_cast<async_operation<void>*>(om_async_operation);

                        if (om_async_operation_t)
                        {
                            om_async_operation_t->completed();
                        }
                        else
                        {
                            // This can occur when the async_operation_base has gone out of scope.
                            print(PRI2, "%p: TcpClientCo1::handle_timer(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
                        }
                    }
                );
#endif
                // End extra code compared with tcpclientco.cpp

                emit connectedSig();
                retVal = true;
            }
            else
            {
                qCritical() << Q_FUNC_INFO << "could not allocate connection info object";
            }
        }
        else
        {
            if (true)
            {
                QAbstractSocket::SocketError error = socket->error();
                QAbstractSocket::SocketState state = socket->state();
                qDebug() << Q_FUNC_INFO;
                qDebug() << "    Error = " << error;
                qDebug() << "    State = " << state;
                qDebug() << "    Could not connect to server...";
            }

            closeConnection(socket);

            // Start timer to reconnect to server
            qDebug() << Q_FUNC_INFO << "Starting timer";
            m_timer.start(m_reconnectTimeout);
        }
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "could not allocate QTcpSocket object";
    }
    return retVal;
}

/**
 * @brief TcpClientCo1::disconnectFromServer is called from the business logic class to disconnect from server
 */
void TcpClientCo1::disconnectFromServer()
{
    qDebug() << Q_FUNC_INFO << m_name;

    foreach (ConnectionInfo *connectionInfo, m_connectionInfoList)
    {
        disconnect(connectionInfo->m_connection_ReadyRead);
        disconnect(connectionInfo->m_connection_disconnected);
        disconnect(connectionInfo->m_connection_stateChanged);
        closeConnection(connectionInfo->m_socket);
    }
    m_connectionInfoList.clear();
    // Stop the timer for in case it was running to try to reconnect
    m_timer.stop();
}

/**
 * @brief TCPClient::sendMessage
 * @param message
 */
void TcpClientCo1::sendMessage(QByteArray& message)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << message.length() << message;

    foreach (ConnectionInfo *connectionInfo, m_connectionInfoList)
    {
        qInfo() << connectionInfo->m_socket;
        if (connectionInfo->m_socket)
        {
            qint64 nrBytesWritten = connectionInfo->m_socket->write(message);
            if (nrBytesWritten == -1)
                qWarning() << Q_FUNC_INFO << "write returned -1";
            else if (nrBytesWritten < message.length())
                qWarning() << Q_FUNC_INFO << "only" << nrBytesWritten << "of" << message.length() << "bytes written";
            bool res = connectionInfo->m_socket->flush();
            // We get a lot of these messages when the network connection is unavailable for a long time
            if (!res)
                qInfo() << Q_FUNC_INFO << "flush returned false";
        }
    }
}

/**
 * @brief TcpClientCo1::readyReadTcp reads all data that has arrived on the socket
 *
 */
void TcpClientCo1::readyReadTcp()
{
    print(PRI2, "%p: TcpClientCo1::readyReadTcp(): m_name = %s\n", this, m_name.toStdString().c_str());

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket)
    {
        QByteArray data = socket->readAll();
        readyReadTcpCo(data);
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpClientCo1::disconnectedServer deals with a disconnected server.
 */
void TcpClientCo1::disconnectedServer()
{
    qInfo() << Q_FUNC_INFO << m_name;
    qDebug() << Q_FUNC_INFO << "Disconnected from server";

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket)
    {
        qDebug() << "Socket disconnected " << socket;
        qDebug() << "Socket parent       " << socket->parent();

        QList<ConnectionInfo*> toRemoveList;
        int count = 0;
        foreach (ConnectionInfo* connectionInfo, m_connectionInfoList)
        {
            if (connectionInfo->m_socket == socket)
            {
                disconnect(connectionInfo->m_connection_ReadyRead);
                disconnect(connectionInfo->m_connection_disconnected);
                disconnect(connectionInfo->m_connection_stateChanged);
                toRemoveList.append(connectionInfo);
                count++;
            }
        }

        if (count == 0)
            qCritical() << Q_FUNC_INFO << "socket not found in connection info list";
        if (count > 1)
            qCritical() << Q_FUNC_INFO << "socket found" << count << "times in connection info list";

        foreach (ConnectionInfo* connectionInfo, m_connectionInfoList)
        {
            m_connectionInfoList.removeOne(connectionInfo);
            delete connectionInfo;
        }
        toRemoveList.clear();

        closeConnection(socket);

        if (m_autoConnect)
        {
            // Start timer to reconnect to server
            qDebug() << Q_FUNC_INFO << "Starting timer";
            m_timer.start(m_reconnectTimeoutAfterDisconnect);
        }

        emit disconnectedServerSig();
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpClientCo1::stateChanged
 * @param socketState
 */
void TcpClientCo1::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "Socket state changed to: " << socketState;
}

/**
 * @brief TcpClientCo1::closeConnection
 * @param socket
 */
void TcpClientCo1::closeConnection(QTcpSocket *socket)
{
    qDebug() << Q_FUNC_INFO << m_name << "socket = " << socket;

    if (socket)
    {
        socket->disconnectFromHost();   // necessary?

        socket->close();
        // deleteLater() will schedule the object delete through the event loop so that any pending events for the object
        // will be removed from the event queue and it can be safely deleted.
        socket->deleteLater();
    }
}

/**
 * @brief TcpClientCo1::readyReadTcpCo
 * @param data
 */
void TcpClientCo1::readyReadTcpCo(QByteArray& data)
{
    if (m_selectImplementation == 1)
        readyReadTcpCo1(data);
    else if (m_selectImplementation == 2)
        readyReadTcpCo2(data);
    else
        readyReadTcpCo1(data);
}

/**
 * @brief TcpClientCo1::readyReadTcpCo1 collects the received bytes to compose a message.
 * When a complete message has arrived, readyReadTcpCo1 emits a signal responseReceivedSig.
 * The coroutine related code will connect a lambda slot function to this signal.
 * @param data
 */
void TcpClientCo1::readyReadTcpCo1(QByteArray& data)
{
    print(PRI2, "%p: TcpClientCo1::readyReadTcpCo1(): m_name = %s, data.length() = %d\n",
          this,
          m_name.toStdString().c_str(),
          data.length());

    if (m_data.length() != 0)
        qInfo() << Q_FUNC_INFO << "m_data.length() = " << m_data.length();

    QByteArray data2;
    m_data += data;
    if (m_message.composeMessage(m_data, data2))
    {
        if (m_message.checkMessage())
        {
            // At the moment do not take the result into account.
        }
        else
        {
            qWarning() << Q_FUNC_INFO << "received incorrect message";
        }

        // Are there any bytes yet for a second message?
        if (data2.length() != 0)
            qInfo() << Q_FUNC_INFO << "data2.length() = " << data2.length();

        m_data = data2;
        if (m_data.length() != 0)
        {
            // There are bytes for a second message
            qInfo() << Q_FUNC_INFO << "starting timer";
            m_receiveTimer.start(0);
            // When the timer expires (which is immediately),
            // readyReadTcpCo (and readyReadTcpCo1) will be called again.
            // This "indirection" is used to first return to the Qt event loop
            // instead of reading all remaining bytes from the same slot function call.
        }

        QByteArray msg = m_message.content();

        print(PRI2, "%p: TcpClientCo1::readyReadTcpCo1(): emitting signal\n", this);
        emit responseReceivedSig(msg);
    }
}

/**
 * @brief TcpClientCo1::readyReadTcpCo2 collects the received bytes to compose a message.
 * When a complete message has arrived, readyReadTcpCo1 emits a signal responseReceivedSig.
 * The coroutine related code will connect a lambda slot function to this signal.
 * In contrast to readyReadTcpCo1, readyReadTcpCo2 uses a variant of 
 * composeMessage that can be called in a loop.
 * @param data
 */
void TcpClientCo1::readyReadTcpCo2(QByteArray& data)
{
    qInfo() << Q_FUNC_INFO;
    qInfo() << data.length() << data;

    int length = data.length();
    int index = 0;
    while (m_message.composeMessage(data, length, index))
    {
        if (m_message.checkMessage())
        {
            // At the moment do not take the result into account.
        }
        else
        {
            qWarning() << Q_FUNC_INFO << "received incorrect message";
        }

        QByteArray msg = m_message.content();
        emit responseReceivedSig(msg);
    } // while
}


/**
 * @brief TcpClientCo1::start_reading
 * @return
 */
async_operation<QByteArray> TcpClientCo1::start_reading()    // no doDisconnect parameter compared with tcpclientco.cpp
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo1::start_reading(): index = %d\n", this, index);
    async_operation<QByteArray> ret{ this, index };         // no doDisconnect parameter compared with tcpclientco.cpp
    start_reading_impl(index);
    return ret;
}

/**
 * @brief TcpClientCo1::start_reading_impl uses Qt's connect to associate 
 * the TcpClientCo1::responseReceivedSig signal function emitted by itself
 * with a lambda that is used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal
 * at the end of its invocation if doDisconnect equals true (this is the default value).
 * @param idx
 */
void TcpClientCo1::start_reading_impl(const int idx)        // no doDisconnect parameter compared with tcpclientco.cpp
{
    print(PRI2, "%p: TcpClientCo1::start_reading_impl(): idx = %d, operation = %p\n", this, idx, get_async_operation(idx));

    m_index_read = idx;            // New statement compared with tcpclientco.cpp
    
    // Original code from tcpclientco.cpp has been moved to connectToServer(), see above
}

/**
 * @brief TcpClientCo1::stop_reading
 * @param idx
 */
void TcpClientCo1::stop_reading(int idx)
{
    // Original code from tcplientco.cpp has been removed.
    (void) idx;
}

/**
 * @brief TcpClientCo1::start_timer
 * @param timer
 * @param ms
 * @return
 */
async_operation<void> TcpClientCo1::start_timer(QTimer& timer, int ms)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo1::start_timer(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief TcpClientCo1::start_timer_impl uses Qt's connect to associate 
 * the QTimer::timeout signal function emitted by tmr
 * with a lambda that is used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal
 * at the end of its invocation.
 * @param idx
 * @param tmr
 * @param ms
 */
void TcpClientCo1::start_timer_impl(const int idx, QTimer& tmr, int ms)
{
    print(PRI2, "%p: TcpClientCo1::start_timer_impl(): idx = %d, operation = %p\n", this, idx, get_async_operation(idx));

    tmr.start(ms);

    m_connections[idx] = connect(&tmr, &QTimer::timeout,
        [this, idx]()
        {
            print(PRI2, "%p: TcpClientCo1::handle_timer() lambda: idx = %d\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpClientCo1::handle_timer(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }

            if (!disconnect(m_connections[idx]))
            {
                print(PRI1, "%p: TcpClientCo1::handle_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
            }
        }
    );
}

/**
  * @brief TcpClientCo1::stop_timer
  * @param idx
  */
void TcpClientCo1::stop_timer(int idx)
{
    if (!disconnect(m_connections[idx]))
    {
        print(PRI1, "%p: TcpClientCo1::stop_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
    }
}

/**
 * @brief TcpClientCo1::start_connecting
 * @param serverIpAddress
 * @param port
 * @return
 */
async_operation<void> TcpClientCo1::start_connecting(QString& serverIpAddress, quint16 port)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo1::start_connecting(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_connecting_impl(index, serverIpAddress, port);
    return ret;
}

/**
 * @brief TcpClientCo1::start_connecting_impl uses Qt's connect to associate 
 * the TcpClientCo1::connectedSig signal function emitted by itself
 * with a lambda that is used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal function.
 * @param idx
 * @param serverIpAddress
 * @param port
 */
void TcpClientCo1::start_connecting_impl(const int idx, QString& serverIpAddress, quint16 port)
{
    print(PRI2, "%p: TcpClientCo1::start_connecting_impl(): idx = %d, operation = %p\n", this, idx, get_async_operation(idx));

    m_index_connect = idx;        // New statement compared with tcpclientco.cpp

    // Original code from tcpclientco.cpp has been moved to connectToServer(), see above
    
    //connectToServer(serverIpAddress, port);

    m_serverIPaddress = serverIpAddress;
    m_serverPort = port;
    m_timer.start(10);
}
