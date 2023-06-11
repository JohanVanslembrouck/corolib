/**
 * @file tcpserver.cpp
 * @brief TCP server class built around QTcpServer.
 * Does not use coroutines.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QNetworkInterface>
#include "tcpserver.h"

/**
 * @brief TcpServer::TcpServer
 * @param name
 */
TcpServer::TcpServer(const QString& name)
    : m_name(name)
{
     qInfo() << Q_FUNC_INFO << m_name;
}

/**
 * @brief TcpServer::configure
 */
void TcpServer::configure()
{
    qInfo() << Q_FUNC_INFO << m_name;

    connect(&m_TcpServer, &QTcpServer::newConnection, this, &TcpServer::newTCPConnection);
    connect(&m_TcpServer, &QTcpServer::acceptError,   this, &TcpServer::acceptError);
}

/**
 * @brief TcpServer::startListening
 * @param port
 */
void TcpServer::startListening(quint16 port)
{
    qInfo() << Q_FUNC_INFO << m_name;

    if (!m_TcpServer.listen(QHostAddress::Any, port))
    {
        qCritical() << Q_FUNC_INFO << "Start listen port" << port << "failed";
        m_TcpServer.close();
        exit(0);
    }
    else
    {
        foreach (const QNetworkInterface &netInterface, QNetworkInterface::allInterfaces())
        {
            QNetworkInterface::InterfaceFlags flags = netInterface.flags();
            if ((bool)(flags & QNetworkInterface::IsRunning) && !(bool)(flags & QNetworkInterface::IsLoopBack))
            {
                foreach (const QNetworkAddressEntry &address, netInterface.addressEntries())
                {
                    if (address.ip().protocol() == QAbstractSocket::IPv4Protocol)
                    {
                        qDebug() << Q_FUNC_INFO << address.ip().toString() <<  " on port" << port;
                    }
                }
            }
        }
    }
    qInfo() << "Listening on " << m_TcpServer.serverAddress() << ":" << m_TcpServer.serverPort();
}

/**
 * @brief TcpServer::enableKeepAlive
 * @param socket
 */
void TcpServer::enableKeepAlive(QTcpSocket *socket)
{
    if (socket)
    {
        socket->setSocketOption(QAbstractSocket::KeepAliveOption, true);
    }
}

/**
 * @brief TcpServer::newTCPConnection
 */
void TcpServer::newTCPConnection()
{
    qInfo() << Q_FUNC_INFO << m_name;

    QTcpSocket *socket = m_TcpServer.nextPendingConnection();
    if (socket)
    {
        enableKeepAlive(socket);

        if (true)   // Could be disabled for release version
        {
            // Get and print information on the new connection
            QHostAddress peerHostAddress = socket->peerAddress();
            QString peerName = socket->peerName();
            quint16 peerPort = socket->peerPort();
            qDebug() << Q_FUNC_INFO;
            qDebug() << "    Connection from " << socket;
            qDebug() << "    peerHostAddress = " << peerHostAddress.toString();
            qDebug() << "    peerName = " << peerName;
            qDebug() << "    peerPort = " << peerPort;
        }

        //m_socketList.append(socket);

        ConnectionInfo *connectionInfo = new ConnectionInfo;
        if (connectionInfo)
        {
            connectionInfo->m_socket = socket;
            qDebug() << Q_FUNC_INFO << connectionInfo->m_socket;
            connectionInfo->m_connection_ReadyRead    = connect(socket, &QTcpSocket::readyRead,    this, &TcpServer::readyReadTcp);
            connectionInfo->m_connection_disconnected = connect(socket, &QTcpSocket::disconnected, this, &TcpServer::disconnectedClient);
            connectionInfo->m_connection_stateChanged = connect(socket, &QTcpSocket::stateChanged, this, &TcpServer::stateChanged);
            connectionInfo->m_closed = false;
            m_connectionInfoList.append(connectionInfo);
        }
        else
        {
            qCritical() << Q_FUNC_INFO << "could not allocate connection info object";
        }

        emit newTCPConnectionSig();
    }
    else
    {
         qCritical() << Q_FUNC_INFO << "nextPendingConnection returned null pointer";
    }
}

/**
 * @brief TcpServer::sendMessage writes the bytes of a message onto a TCP socket
 * @param sock
 * @param message
 */
void TcpServer::sendMessage(QTcpSocket* sock, QByteArray& message)
{
    qInfo() << Q_FUNC_INFO;
    qInfo() << message.length() << message;

    for (ConnectionInfo *connectionInfo : m_connectionInfoList)
    {
        if (connectionInfo->m_socket == sock)
        {
            qInfo() << Q_FUNC_INFO << connectionInfo->m_socket;
            qint64 nrBytesWritten = connectionInfo->m_socket->write(message);
            if (nrBytesWritten == -1)
                qWarning() << Q_FUNC_INFO << "write returned -1";
            else if (nrBytesWritten < message.length())
                qWarning() << Q_FUNC_INFO << "only" << nrBytesWritten << "of" << message.length() << "bytes written";
            bool res = connectionInfo->m_socket->flush();
            // We get a lot of these messages when the network connection is unavailable for a long time
            if (!res)
                qInfo() << Q_FUNC_INFO << "flush returned false";
            break;
        }
    }
}

/**
 * @brief TcpServer::readyReadTcp
 *
 */
void TcpServer::readyReadTcp()
{
    qInfo() << Q_FUNC_INFO << m_name;

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (socket)
    {
        QByteArray data = socket->readAll();
        qInfo() << socket << ":" << data;
        qInfo() << data << data.length();

        if (data.length() < 650)
            qInfo() << data.length() << data;
        else
            qInfo() << data.length();    // Only print the length of long messages, not the content

        bool found = false;
        for (ConnectionInfo* connectionInfo : m_connectionInfoList)
        {
            if (connectionInfo->m_socket == socket)
            {
                qInfo() << Q_FUNC_INFO << connectionInfo->m_socket;
                found = true;
                break;
            }
        }
        if (!found)
            qDebug() << Q_FUNC_INFO << "socket not found in m_connectionInfoList";

        emit readyReadTcpSig(socket, data);
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpServer::disconnectedClient
 *
 */
void TcpServer::disconnectedClient()
{
    qDebug() << Q_FUNC_INFO << m_name;

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

        if (count != 1)
            qCritical() << Q_FUNC_INFO <<  "connection found " << count << " times in list (instead of once)";

        foreach (ConnectionInfo* connectionInfo, m_connectionInfoList)
        {
            m_connectionInfoList.removeOne(connectionInfo);
            delete connectionInfo;
        }
        toRemoveList.clear();

        closeConnection(socket);

        emit disconnectedClientSig();
    }
    else
    {
        qCritical() << Q_FUNC_INFO << "sender() returned null pointer";
    }
}

/**
 * @brief TcpServer::stateChanged
 * @param socketState
 */
void TcpServer::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "Socket changed to: " << socketState;
}

/**
 * @brief TcpServer::acceptError
 * @param socketError
 */
void TcpServer::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << Q_FUNC_INFO << m_name;
    qInfo() << "acceptError: " << socketError;
}

/**
 * @brief TcpServer::closeConnection
 * NOT YET USED
 */
void TcpServer::closeConnection()
{
    qInfo() << Q_FUNC_INFO << m_name;

    // Should only mark one connection as closed. TBC.
    for (ConnectionInfo *connInfo1 : m_connectionInfoList)
    {
        closeConnection(connInfo1->m_socket);
    }
    m_connectionInfoList.clear();
}

/**
 * @brief TcpServer::closeConnection
 * @param socket
 */
void TcpServer::closeConnection(QTcpSocket *socket)
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
