/**
 * @file tcpclient00.cpp
 * @brief 
 * Implementation of the first TCP client appliation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "tcpclient00.h"
#include "tcpconfig.h"

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision

using namespace std;

static int nr_message_lengths = 10;

/**
 * @brief TcpClient00::TcpClient00
 * @param parent
 */
TcpClient00::TcpClient00(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_server(configuration.m_server)

    , m_errorCounter(0)
    , m_selection(0)
    //, m_loop(configuration.m_selectMeasurementLoop)

    , m_timerConnectToServer(this)
    , m_timerStartSending(this)
    , m_timerNoResponse(this)

    , m_tcpClient("",
                  true,
                  configuration.m_waitForConnectionTimeout,
                  configuration.m_reconnectTimeout,
                  configuration.m_reconnectTimeoutAfterDisconnect)

    , m_message(check)
{
    qInfo() << Q_FUNC_INFO << m_server.m_ipAddress << ":" << m_server.m_port;

    m_timerConnectToServer.setSingleShot(true);
    m_timerStartSending.setSingleShot(true);
    m_timerNoResponse.setSingleShot(true);

    connect(&m_timerConnectToServer,&QTimer::timeout, this, &TcpClient00::connectToServer);
    connect(&m_timerStartSending,   &QTimer::timeout, this, &TcpClient00::sendTCPStart);
    connect(&m_timerNoResponse,     &QTimer::timeout, this, &TcpClient00::noResponseReceived);
    connect(&m_tcpClient,           &TcpClient::connectedSig, this, &TcpClient00::connected);

    nr_message_lengths = configuration.m_numberMessages;

    configureTCP();
}

/**
 * @brief TcpClient00::configureTCP
 */
void TcpClient00::configureTCP()
{
    qInfo() << Q_FUNC_INFO;

    m_tcpClient.configure();
    connect(&m_tcpClient, &TcpClient::readyReadTcpSig, this, &TcpClient00::readyReadTcp);
}

/**
 * @brief TcpClient00::start
 */
void TcpClient00::start()
{
    qInfo() << Q_FUNC_INFO;
    connectToServer();
}

/**
 * @brief TcpClient00::addErrorMessage
 * @param message
 */
void TcpClient00::addErrorMessage(const QString &message)
{
    qDebug() << "ERROR:" << message;
}

/**
 * @brief TcpClient00::addWarningMessage
 * @param message
 */
void TcpClient00::addWarningMessage(const QString &message)
{
    qDebug() << "WARNING" << message;
}

/**
 * @brief TcpClient00::addInfoMessage
 * @param message
 */
void TcpClient00::addInfoMessage(const QString &message)
{
    qDebug() << "INFO:" << message;
}

/**
 * @brief TcpClient00::connectToServer
 */
void TcpClient00::connectToServer()
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    qInfo() << "m_server.m_ipAddress = " << m_server.m_ipAddress << ", m_server.m_port = " << m_server.m_port;

    bool result = m_tcpClient.connectToServer(m_server.m_ipAddress, m_server.m_port);
    if (!result)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";
}

/**
 * @brief TcpClient00::connected
 */
void TcpClient00::connected()
{
    qDebug() << Q_FUNC_INFO;
    sendTCPStart();
}

/**
 * @brief TcpClient00::noResponseReceived
 */
void TcpClient00::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient00::sendTCPStart
 */
void TcpClient00::sendTCPStart()
{
    m_selection++;
    qInfo() << Q_FUNC_INFO;

    m_counter = 0;

    if (m_selection < nr_message_lengths)
    {
        m_start = chrono::high_resolution_clock::now();
        QByteArray data = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data);
        m_timerNoResponse.start(10 * data.length());
    }
}

/**
 * @brief TcpClient00::prepareMessage
 * @param selection
 * @return
 */
QByteArray TcpClient00::prepareMessage(int selection)
{
    //qDebug() << Q_FUNC_INFO << selection;
    QByteArray data = composeMessage(selection, configuration.m_step);

    ProtocolMessage message(USE_CRC);
    message.createMessage(data);

    QByteArray data2 = message.content();

    // Some extra test
    if (data2[0] != STX || data2[data2.length() - 1] != ETX)
    {
        qDebug() << "Wrong message termination";
    }

    return data2;
}

/**
 * @brief TcpClient00::quit
 */
void TcpClient00::quit()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient00::acceptError
 * @param socketError
 */
void TcpClient00::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << "acceptError: " << socketError;
}

/**
 * @brief TcpClient00::calculateElapsedTime
 */
void TcpClient00::calculateElapsedTime(std::chrono::high_resolution_clock::time_point start)
{
    qInfo() << "End of cycle";
    std::chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();

    // Calculating total time taken by the program.
    double time_taken =
        chrono::duration_cast<chrono::nanoseconds>(end - start).count();

    // Time per transaction
    // --------------------
    double time_taken2 = time_taken / configuration.m_numberTransactions;
    time_taken2 *= 1e-6;

    cout << setprecision(6);
    cout << "Time taken by 1 transaction (length = " << m_message.length() << ") (averaged over " << configuration.m_numberTransactions
         << " transactions) is : " << fixed << time_taken2 << " msec" << endl;
}

/**
 * @brief TcpClient00::readyReadTcp
 * @param data
 */
void TcpClient00::readyReadTcp(QByteArray& data)
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

        qInfo() << "counter:" << m_counter << "received message:" << m_message.content();
        if (++m_counter < configuration.m_numberTransactions)
        {
            QByteArray data2 = prepareMessage(m_selection);
            qInfo() << "sending message:" << data2;
            m_tcpClient.sendMessage(data2);
            m_timerNoResponse.start(10 * data2.length());
        }
        else
        {
            calculateElapsedTime(m_start);
            m_timerStartSending.start(100);
        }
    } // while
}

/**
 * @brief TcpClient00::errorOccurred
 * @param socketError
 */
void TcpClient00::errorOccurred(QAbstractSocket::SocketError socketError)
{
    qDebug() << Q_FUNC_INFO << socketError;
}

/**
 * @brief TcpClient00::hostFound
 */
void TcpClient00::hostFound()
{

}

/**
 * @brief TcpClient00::stateChanged
 * @param socketState
 */
void TcpClient00::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << "Socket changed to: " << socketState;
}
