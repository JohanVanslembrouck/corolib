/**
 * @file tcpclient.cpp
 * @brief Implementation of a TCP client class.
 * Does not use coroutines.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclient.h"

/**
 * @brief TcpClient::TcpClient
 * @param name
 * @param autoConnect
 * @param waitForConnectionTimeout
 * @param reconnectTimeout
 * @param reconnectTimeoutAfterDisconnect
 */
TcpClient::TcpClient(const QString& name,
                     bool autoConnect,
                     qint32 waitForConnectionTimeout,
                     qint32 reconnectTimeout,
                     qint32 reconnectTimeoutAfterDisconnect)
    : m_timer(this)
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
void TcpClient::configure()
{
    qInfo() << Q_FUNC_INFO << m_name;

    // Client starts timer to reconnect to server if connection was lost.
    m_timer.setSingleShot(true);
    connect(&m_timer, &QTimer::timeout, this, &TcpClient::connectToServerTimed);
}

/**
 * @brief TcpClient::connectToServerTimed
 */
void TcpClient::connectToServerTimed()
{
    qInfo() << Q_FUNC_INFO << m_name;

    connectToServer(m_serverIPaddress, m_serverPort);
}

/**
 * @brief TcpClient::enableKeepAlive
 * @param socket
 */
void TcpClient::enableKeepAlive(QTcpSocket *socket)
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
 * @brief TcpClient::connectToServer
 * @param serverIPaddress
 * @param serverPort
 * @return
 */
bool TcpClient::connectToServer(QString& serverIPaddress, quint16 serverPort)
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
                connectionInfo->m_connection_ReadyRead    = connect(socket, &QTcpSocket::readyRead,    this, &TcpClient::readyReadTcp);
                connectionInfo->m_connection_disconnected = connect(socket, &QTcpSocket::disconnected, this, &TcpClient::disconnectedServer);
                connectionInfo->m_connection_stateChanged = connect(socket, &QTcpSocket::stateChanged, this, &TcpClient::stateChanged);
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
 * @brief TcpClient::disconnectFromServer
 * Function called from business logic class to disconnect from server
 */
void TcpClient::disconnectFromServer()
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
void TcpClient::sendMessage(QByteArray& message)
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
 * @brief TcpClient::readyReadTcp
 *
 */
void TcpClient::readyReadTcp()
{
    qInfo() << Q_FUNC_INFO << m_name;

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket)
    {
        QByteArray data = socket->readAll();

        if (data.length() < 650)
            qInfo() << data.length() << data;
        else
            qInfo() << data.length();

        emit readyReadTcpSig(data);
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpClient::disconnectedServer
 * is used exclusively on the client side to deal with a disconnected server.
 */
void TcpClient::disconnectedServer()
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
            qDebug() << Q_FUNC_INFO << "Starting timer" << Qt::endl;
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
void TcpClient::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "Socket state changed to: " << socketState;
}

/**
 * @brief TcpClient::closeConnection
 * @param socket
 */
void TcpClient::closeConnection(QTcpSocket *socket)
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
