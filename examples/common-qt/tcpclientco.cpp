/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclientco.h"

/**
 * @brief TcpClientCo::TcpClientCo
 * @param name
 * @param autoConnect
 * @param waitForConnectionTimeout
 * @param reconnectTimeout
 * @param reconnectTimeoutAfterDisconnect
 */
TcpClientCo::TcpClientCo(bool useCoroutines,
                         qint32 selectImplementation,
                         const QString& name,
                         bool autoConnect,
                         qint32 waitForConnectionTimeout,
                         qint32 reconnectTimeout,
                         qint32 reconnectTimeoutAfterDisconnect)
    : m_useCoroutines(useCoroutines)
    , m_selectImplementation(selectImplementation)
    , m_timer(this)
    , m_transmitTimer(this)
    , m_autoConnect(autoConnect)
    , m_name(name)
    , m_waitForConnectionTimeout(waitForConnectionTimeout)
    , m_reconnectTimeout(reconnectTimeout)
    , m_reconnectTimeoutAfterDisconnect(reconnectTimeoutAfterDisconnect)
{
    qInfo() << Q_FUNC_INFO << m_name << ", autoConnect = " << autoConnect;
}

/**
 * @brief TCPClient::configure
 */
void TcpClientCo::configure()
{
    qInfo() << Q_FUNC_INFO << m_name;

    // Client starts timer to reconnect to server if connection was lost.
    m_timer.setSingleShot(true);
    m_transmitTimer.setSingleShot(true);

    connect(&m_timer,         &QTimer::timeout, this, &TcpClientCo::connectToServerTimed);
    connect(&m_transmitTimer, &QTimer::timeout, this, &TcpClientCo::transmitTimed);
}

/**
 * @brief TcpClientCo::connectToServerTimed
 */
void TcpClientCo::connectToServerTimed()
{
    qInfo() << Q_FUNC_INFO << m_name;

    connectToServer(m_serverIPaddress, m_serverPort);
}

/**
 * @brief TcpClientCo::transmitTimed
 */
void TcpClientCo::transmitTimed()
{
    qInfo() << Q_FUNC_INFO << m_name;

    QByteArray empty;
    readyReadTcpCo(empty);
}

/**
 * @brief TcpClientCo::enableKeepAlive
 * @param socket
 */
void TcpClientCo::enableKeepAlive(QTcpSocket *socket)
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
 * @brief TcpClientCo::connectToServer
 * @param serverIPaddress
 * @param serverPort
 * @return
 */
bool TcpClientCo::connectToServer(QString& serverIPaddress, quint16 serverPort)
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
                connectionInfo->m_connection_ReadyRead    = connect(socket, &QTcpSocket::readyRead,    this, &TcpClientCo::readyReadTcp);
                connectionInfo->m_connection_disconnected = connect(socket, &QTcpSocket::disconnected, this, &TcpClientCo::disconnectedServer);
                connectionInfo->m_connection_stateChanged = connect(socket, &QTcpSocket::stateChanged, this, &TcpClientCo::stateChanged);
                m_connectionInfoList.append(connectionInfo);

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
 * @brief TcpClientCo::disconnectFromServer
 * Function called from business logic class to disconnect from server
 */
void TcpClientCo::disconnectFromServer()
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
void TcpClientCo::sendMessage(QByteArray& message)
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
 * @brief TcpClientCo::readyReadTcp
 *
 */
void TcpClientCo::readyReadTcp()
{
    print(PRI2, "%p: TcpClientCo::readyReadTcp(): m_name = %s\n", this, m_name.toStdString().c_str());

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket)
    {
        QByteArray data = socket->readAll();
        if (m_useCoroutines)
            readyReadTcpCo(data);
        else
            emit readyReadTcpSig(data);
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpClientCo::disconnectedServer
 * is used exclusively on the client side to deal with a disconnected server.
 */
void TcpClientCo::disconnectedServer()
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
 * @brief TCPClient::stateChanged
 * @param socketState
 */
void TcpClientCo::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "Socket state changed to: " << socketState;
}

/**
 * @brief TcpClientCo::closeConnection
 * @param socket
 */
void TcpClientCo::closeConnection(QTcpSocket *socket)
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
 * @brief TcpClientCo::readyReadTcpCo
 * @param data
 */
void TcpClientCo::readyReadTcpCo(QByteArray& data)
{
    if (m_selectImplementation == 1)
        readyReadTcpCo1(data);
    else if (m_selectImplementation == 2)
        readyReadTcpCo2(data);
    else
        readyReadTcpCo1(data);
}

/**
 * @brief TcpClientCo::readyReadTcpCo1
 * @param data
 */
void TcpClientCo::readyReadTcpCo1(QByteArray& data)
{
    print(PRI2, "%p: TcpClientCo::readyReadTcpCo(): m_name = %s, data.length() = %d\n",
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

        if (data2.length() != 0)
            qInfo() << Q_FUNC_INFO << "data2.length() = " << data2.length();

        m_data = data2;
        if (m_data.length() != 0)
        {
            qInfo() << Q_FUNC_INFO << "starting timer";
            m_transmitTimer.start(0);
        }

        QByteArray msg = m_message.content();

        print(PRI2, "%p: TcpClientCo::readyReadTcpCo(): emitting signal\n", this);
        emit responseReceivedSig(msg);
    }
}

/**
 * @brief TcpClientCo::readyReadTcpCo2
 * @param data
 */
void TcpClientCo::readyReadTcpCo2(QByteArray& data)
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
 * @brief TcpClientCo::start_reading
 * @return
 */
async_operation<QByteArray> TcpClientCo::start_reading(bool doDisconnect)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo::start_reading(): index = %d\n", this, index);
    async_operation<QByteArray> ret{ this, index };
    start_reading_impl(index, doDisconnect);
    return ret;
}

/**
 * @brief TcpClientCo::start_reading_impl
 * @param idx
 */
void TcpClientCo::start_reading_impl(const int idx, bool doDisconnect)
{
    print(PRI2, "%p: TcpClientCo::start_reading_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(this, &TcpClientCo::responseReceivedSig,
        [this, idx, doDisconnect](QByteArray msg)
        {
            print(PRI2, "%p: TcpClientCo::handle_read() lambda: idx = %d\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<QByteArray>* om_async_operation_t =
                dynamic_cast<async_operation<QByteArray>*>(om_async_operation);

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

            if (doDisconnect)
            {
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: TcpClientCo::handle_read(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

/**
 * @brief TcpClientCo::stop_reading
 * @param idx
 */
void TcpClientCo::stop_reading(int idx)
{
    if (!disconnect(m_connections[idx]))
    {
        print(PRI1, "%p: TcpClientCo::stop_reading(): idx = %d, Warning: disconnect failed\n", this, idx);
    }
}

/**
 * @brief TcpClientCo::start_timer
 * @param timer
 * @param ms
 * @return
 */
async_operation<void> TcpClientCo::start_timer(QTimer& timer, int ms)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo::start_timer(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief TcpClientCo::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void TcpClientCo::start_timer_impl(const int idx, QTimer& tmr, int ms)
{
    print(PRI2, "%p: TcpClientCo::start_timer_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    tmr.start(ms);

    m_connections[idx] = connect(&tmr, &QTimer::timeout,
        [this, idx]()
        {
            print(PRI2, "%p: TcpClientCo::handle_timer() lambda: idx = %d\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpClientCo::handle_timer(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }

            if (!disconnect(m_connections[idx]))
            {
                print(PRI1, "%p: TcpClientCo::handle_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
            }
        }
    );
}

/**
  * @brief TcpClientCo::stop_timer
  * @param idx
  */
void TcpClientCo::stop_timer(int idx)
{
    if (!disconnect(m_connections[idx]))
    {
        print(PRI1, "%p: TcpClientCo::stop_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
    }
}

/**
 * @brief TcpClientCo::start_connecting
 * @param serverIpAddress
 * @param port
 * @return
 */
async_operation<void> TcpClientCo::start_connecting(QString& serverIpAddress, quint16 port)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClientCo::start_connecting(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_connecting_impl(index, serverIpAddress, port);
    return ret;
}

/**
 * @brief TcpClientCo::start_connecting_impl
 * @param idx
 * @param serverIpAddress
 * @param port
 */
void TcpClientCo::start_connecting_impl(const int idx, QString& serverIpAddress, quint16 port)
{
    print(PRI2, "%p: TcpClientCo::start_connecting_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(this, &TcpClientCo::connectedSig,
        [this, idx]()
        {
            print(PRI2, "%p: TcpClientCo::handle_connect() lambda: idx = %d\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpClientCo::handle_connect(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }

            if (!disconnect(m_connections[idx]))
            {
                print(PRI1, "%p: TcpClientCo::handle_connect(): idx = %d, Warning: disconnect failed\n", this, idx);
            }
        }
    );

    //connectToServer(serverIpAddress, port);

    m_serverIPaddress = serverIpAddress;
    m_serverPort = port;
    m_timer.start(10);
}
