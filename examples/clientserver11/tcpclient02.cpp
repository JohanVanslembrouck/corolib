/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclient02.h"
#include "tcpconfig.h"

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision
using namespace std;

const int NR_MESSAGE_LENGTHS = 7;
static int nr_message_lengths = NR_MESSAGE_LENGTHS;

/**
 * @brief TcpClient02::TcpClient02
 * @param parent
 */
TcpClient02::TcpClient02(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_serverName(configuration.m_serverName)
    , m_serverHost(configuration.m_server.m_ipAddress)
    , m_serverPort(configuration.m_server.m_port)

    , m_errorCounter(0)
    , m_selection(0)

    , m_timerConnectToServer(this)
    , m_timerStartSending(this)
    , m_timerNoResponse(this)

    , m_tcpClient1(configuration.m_useCoroutines,
                   "client1",
                   true,
                   configuration.m_waitForConnectionTimeout,
                   configuration.m_reconnectTimeout,
                   configuration.m_reconnectTimeoutAfterDisconnect)

    , m_tcpClient2(configuration.m_useCoroutines,
                   "client2",
                   true,
                   configuration.m_waitForConnectionTimeout,
                   configuration.m_reconnectTimeout,
                   configuration.m_reconnectTimeoutAfterDisconnect)

    , m_message(check)
{
    qInfo() << Q_FUNC_INFO << m_serverHost << ":" << m_serverPort;

    m_timerConnectToServer.setSingleShot(true);
    m_timerStartSending.setSingleShot(true);
    m_timerNoResponse.setSingleShot(true);

    connect(&m_timerConnectToServer,&QTimer::timeout, this, &TcpClient02::connectToServer);
    connect(&m_timerStartSending,   &QTimer::timeout, this, &TcpClient02::sendTCPStart);
    connect(&m_timerNoResponse,     &QTimer::timeout, this, &TcpClient02::noResponseReceived);

    if (configuration.m_latencyMeasurement)
        nr_message_lengths = configuration.m_numberMessages;

    configureTCP();
}

/**
 * @brief TcpClient02::configureTCP
 */
void TcpClient02::configureTCP()
{
    qInfo() << Q_FUNC_INFO;

    m_tcpClient1.configure();
    connect(&m_tcpClient1, &TcpClientCo::readyReadTcpSig, this, &TcpClient02::readyReadTcp);
    m_tcpClient2.configure();
    connect(&m_tcpClient2, &TcpClientCo::readyReadTcpSig, this, &TcpClient02::readyReadTcp);
}

/**
 * @brief TcpClient02::start
 */
void TcpClient02::start()
{
    qInfo() << Q_FUNC_INFO;
    connectToServerDelayed();
}

/**
 * @brief TcpClient02::addErrorMessage
 * @param message
 */
void TcpClient02::addErrorMessage(const QString &message)
{
    qDebug() << "ERROR:" << message;
}

/**
 * @brief TcpClient02::addWarningMessage
 * @param message
 */
void TcpClient02::addWarningMessage(const QString &message)
{
    qDebug() << "WARNING" << message;
}

/**
 * @brief TcpClient02::addInfoMessage
 * @param message
 */
void TcpClient02::addInfoMessage(const QString &message)
{
    qDebug() << "INFO:" << message;
}

/**
 * @brief TcpClient02::connectToServerDelayed
 */
void TcpClient02::connectToServerDelayed()
{
    qInfo() << Q_FUNC_INFO;

    if (configuration.m_startupDelay > 0)
    {
        m_timerConnectToServer.start(configuration.m_startupDelay);
    }
    else
        connectToServer();
}

/**
 * @brief TcpClient02::connectToServer
 */
void TcpClient02::connectToServer()
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    qInfo() << "m_serverHost = " << m_serverHost << ", m_serverPort = " << m_serverPort;

    bool result1 = m_tcpClient1.connectToServer(m_serverHost, m_serverPort);
    if (!result1)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";
    bool result2 = m_tcpClient2.connectToServer(m_serverHost, m_serverPort);
    if (!result2)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";

    if (result1 && result2)
        sendTCPStart();
}

void TcpClient02::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient02::sendTCPStart
 */
void TcpClient02::sendTCPStart()
{
    qInfo() << Q_FUNC_INFO;

    m_selection++;
    qInfo() << Q_FUNC_INFO << m_selection % nr_message_lengths;

    m_counter = 0;

    if (!configuration.m_useCoroutines)
    {
        m_start = chrono::high_resolution_clock::now();
        QByteArray data = prepareMessage(m_selection % nr_message_lengths);
        m_tcpClient1.sendMessage(data);
        m_timerNoResponse.start(10 * data.length());

        //m_tcpClient2.sendMessage(data);
    }
    else
    {
        switch (configuration.m_selectMeasurementLoop)
        {
        case 0:
        {
            async_task<int> si = measurementLoop0();
        }
        break;
        case 1:
        {
            async_task<int> si = measurementLoop1();
        }
        break;
        case 2:
        {
            async_task<int> si = measurementLoop2();
        }
        break;

        case 10:
        {
            async_task<int> si = measurementLoop10();
        }
        break;
        case 11:
        {
            async_task<int> si = measurementLoop11();
        }
        break;
        case 12:
        {
            async_task<int> si = measurementLoop12();
        }
        break;
        case 13:
        {
            async_task<int> si = measurementLoop13();
        }
        break;
        case 14:
        {
            async_task<int> si = measurementLoop14();
        }
        break;
        default:
            qDebug() << "Please select a valid measurement loop";
        }
    }
}

/**
 * @brief TcpClient02::prepareMessage
 * @param selection
 * @return
 */
QByteArray TcpClient02::prepareMessage(int selection)
{
    qInfo() << Q_FUNC_INFO << "selection = " << selection;
    QByteArray data = composeMessage(selection,
                                     configuration.m_latencyMeasurement,
                                     configuration.m_step);

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
 * @brief TcpClient02::quit
 */
void TcpClient02::quit()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient02::acceptError
 * @param socketError
 */
void TcpClient02::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << "acceptError: " << socketError;
}

/**
 * @brief TcpClient02::calculateElapsedTime
 */
void TcpClient02::calculateElapsedTime(int messageLength)
{
    qInfo() << "End of cycle";
    m_end = chrono::high_resolution_clock::now();

    // Calculating total time taken by the program.
    double time_taken =
        chrono::duration_cast<chrono::nanoseconds>(m_end - m_start).count();

    // Time per transaction
    // --------------------
    double time_taken2 = time_taken / configuration.m_numberTransactions;
    time_taken2 *= 1e-6;

    cout << setprecision(6);
    cout << "Time taken by 1 transaction (length = " << messageLength << ") (averaged over " << configuration.m_numberTransactions
         << " transactions) is : " << fixed << time_taken2 << " msec" << endl;
}

/**
 * @brief TcpClient02::readyReadTcp
 * @param data
 */
void TcpClient02::readyReadTcp(QByteArray& data)
{
    qInfo() << Q_FUNC_INFO;
    qInfo() << data.length() << data;

    QByteArray data2;
    while (m_message.composeMessage(data, data2))
    {
        if (m_message.checkMessage())
        {
            // At the moment do not take the result into account.
        }
        else
        {
            qWarning() << Q_FUNC_INFO << "received incorrect message";
        }

        if (configuration.m_useCoroutines)
        {
            emit responseReceivedSig();
            m_timerNoResponse.stop();
        }
        else
        {
            qInfo() << "counter:" << m_counter << "received message:" << m_message.content();
            if (++m_counter < configuration.m_numberTransactions)
            {
                QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
                qInfo() << "sending message:" << data2;
                m_tcpClient1.sendMessage(data2);
                m_timerNoResponse.start(10 * data2.length());
            }
            else
            {
                calculateElapsedTime(m_message.length());
                m_timerStartSending.start(100);
            }
        }

        if (data2.length() == 0)
        {
            break;
        }
        else
        {
            qInfo() << Q_FUNC_INFO << "data contained subsequent message with length" << data2.length();
            data = data2;
            data2.clear();
        }
    } // while
}

/**
 * @brief TcpClient02::connected
 */
void TcpClient02::connected()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient02::errorOccurred
 * @param socketError
 */
void TcpClient02::errorOccurred(QAbstractSocket::SocketError socketError)
{
    qDebug() << Q_FUNC_INFO << socketError;
}

/**
 * @brief TcpClient02::hostFound
 */
void TcpClient02::hostFound()
{

}

/**
 * @brief TcpClient02::stateChanged
 * @param socketState
 */
void TcpClient02::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << "Socket changed to: " << socketState;
}

// Coroutine related
// =================

/**
 * @brief TcpClient02::measurementLoop0
 * @return
 */
async_task<int> TcpClient02::measurementLoop0()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);
        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        qInfo() << dataOut1.length() << ":" << dataOut1;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop1
 * @return
 */
async_task<int> TcpClient02::measurementLoop1()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);

        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data2.length();
        m_tcpClient1.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        async_operation<QByteArray> op2 = m_tcpClient1.start_reading();
        QByteArray dataOut2 = co_await op2;       // Hangs here because both responses were received before

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop2
 * @return
 */
async_task<int> TcpClient02::measurementLoop2()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);

        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data2.length();
        m_tcpClient1.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient1.start_reading();

        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop10
 * @return
 */
async_task<int> TcpClient02::measurementLoop10()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);
        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        m_tcpClient2.sendMessage(data2);
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop11
 * @return
 */
async_task<int> TcpClient02::measurementLoop11()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();

        m_tcpClient1.sendMessage(data1);
        m_tcpClient2.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop12
 * @return
 */
async_task<int> TcpClient02::measurementLoop12()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        m_tcpClient1.sendMessage(data1);
        QByteArray dataOut1 = co_await op1;

        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();
        m_tcpClient2.sendMessage(data2);
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop13
 * @return
 */
async_task<int> TcpClient02::measurementLoop13()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

        m_tcpClient1.sendMessage(data1);
        m_tcpClient2.sendMessage(data2);

        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop14
 * @return
 */
async_task<int> TcpClient02::measurementLoop14()
{
    int msgLength = 0;
    m_start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection % nr_message_lengths);
        QByteArray data2 = prepareMessage(m_selection % nr_message_lengths);
        msgLength = data1.length();

        m_tcpClient1.sendMessage(data1);
        m_tcpClient2.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}
