/**
 * @file tcpserver01.cpp
 * @brief
 * Implementation of the first TCP server application.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QThread>

#include "tcpserver01.h"
#include "tcpconfig.h"

/**
 * @brief TcpServer01::TcpServer01
 * @param parent
 */
TcpServer01::TcpServer01(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_serverHost(configuration.m_server.m_ipAddress)
    , m_serverPort(configuration.m_server.m_port)
    , m_tcpServer()
    , m_message(check)
    , m_errorCounter(0)
{
    qDebug() << Q_FUNC_INFO << m_serverHost << ":" << m_serverPort;

    configureTCP();
}

/**
 * @brief TcpServer01::configureTCP
 */
void TcpServer01::configureTCP()
{
    qInfo() << Q_FUNC_INFO;

    m_tcpServer.configure();
    connect(&m_tcpServer, &TcpServer::readyReadTcpSig,       this, &TcpServer01::readyReadTcp);
    connect(&m_tcpServer, &TcpServer::newTCPConnectionSig,   this, &TcpServer01::newTCPConnection);
    connect(&m_tcpServer, &TcpServer::disconnectedClientSig, this, &TcpServer01::disconnectedTCPClient);
}

/**
 * @brief TcpServer01::start
 * called from main() after having created a TcpServer01 object
 *
 * @note
 * server.listen(QHostAddress::LocalHost, port);
 * accepts only connections from clients using "localhost" or "127.0.0.1",
 * not even if the IP address is the same as that of the computer.
 *
 */
void TcpServer01::start()
{
    qInfo() << Q_FUNC_INFO;
    m_tcpServer.startListening(m_serverPort);
}

/**
 * @brief TcpServer01::addErrorMessage
 * @param message
 */
void TcpServer01::addErrorMessage(const QString &message)
{
    qDebug() << "ERROR:" << message;
}

/**
 * @brief TcpServer01::addWarningMessage
 * @param message
 */
void TcpServer01::addWarningMessage(const QString &message)
{
    qDebug() << "WARNING" << message;
}

/**
 * @brief TcpServer01::addInfoMessage
 * @param message
 */
void TcpServer01::addInfoMessage(const QString &message)
{
    qDebug() << "INFO:" << message;
}


/**
 * @brief TcpServer01::quit
 */
void TcpServer01::quit()
{
    qDebug() << Q_FUNC_INFO;
    //m_tcpserver.close();      TBC
}

/**
 * @brief TcpServer01::acceptError
 * @param socketError
 */
void TcpServer01::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << "acceptError: " << socketError;
}

/**
 * @brief TcpServer01::readyReadTcp receives messages sent by a client application
 * and sends the same message nrRepetitions times as response to the client.
 * @param socket is the socket on which data has arrived
 * @param data is a byte array with the (partial) content of one or more messages
 */
void TcpServer01::readyReadTcp(QTcpSocket* sock, QByteArray& data)
{
    qInfo() << Q_FUNC_INFO;
    qInfo() << data.length() << data;

    int length = data.length();
    int index = 0;
    // Compose one or more messages from the received byte array.
    while (m_message.composeMessage(data, length, index))
    {
        // A complete message is received. Check its content first.
        if (m_message.checkMessage())
        {
            // At the moment do not take the result into account.
        }
        else
        {
            qWarning() << Q_FUNC_INFO << "received incorrect message";
        }

        QByteArray content = m_message.content();
        qInfo() << "TCPIP: " << content;
        // The second byte in the message indicates the number of responses
        // the client application expects:
        int nrRepetitions = content[1];
        for (int i = 0; i < nrRepetitions; i++)
        {
            // Send each response after a delay
            QThread::msleep(configuration.m_delayBeforeReply);
            m_tcpServer.sendMessage(sock, content);
        }
    } // while
}

/**
 * @brief TcpServer01::newTCPConnection
 */
void TcpServer01::newTCPConnection()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer01::disconnectedTCPClient
 */
void TcpServer01::disconnectedTCPClient()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer01::connected
 */
void TcpServer01::connected()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer01::errorOccurred
 * @param socketError
 */
void TcpServer01::errorOccurred(QAbstractSocket::SocketError socketError)
{
    qDebug() << Q_FUNC_INFO << socketError;
}

/**
 * @brief TcpServer01::hostFound
 */
void TcpServer01::hostFound()
{

}

/**
 * @brief TcpServer01::stateChanged
 * @param socketState
 */
void TcpServer01::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << "Socket changed to: " << socketState;
}
