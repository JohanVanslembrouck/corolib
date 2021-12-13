/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclient02.h"
#include "tcpconfig.h"

#include "wait_all_awaitable.h"

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision
using namespace std;

static int nr_message_lengths = 10;

/**
 * @brief TcpClient02::TcpClient02
 * @param parent
 */
TcpClient02::TcpClient02(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_errorCounter(0)
    , m_selection(0)
    , m_loop(configuration.m_selectMeasurementLoop)

    , m_timerConnectToServer(this)
    , m_timerStartSending(this)
    , m_timerNoResponse(this)

    , m_tcpClient1(configuration.m_useCoroutines,
                   configuration.m_selectImplementation,
                   "client1",
                   true,
                   configuration.m_waitForConnectionTimeout,
                   configuration.m_reconnectTimeout,
                   configuration.m_reconnectTimeoutAfterDisconnect)

    , m_tcpClient2(configuration.m_useCoroutines,
                   configuration.m_selectImplementation,
                   "client2",
                   true,
                   configuration.m_waitForConnectionTimeout,
                   configuration.m_reconnectTimeout,
                   configuration.m_reconnectTimeoutAfterDisconnect)

    , m_message(check)
{
    qInfo() << Q_FUNC_INFO;

    m_servers[0] = configuration.m_servers[0];
    m_servers[1] = configuration.m_servers[1];

    m_timerConnectToServer.setSingleShot(true);
    m_timerStartSending.setSingleShot(true);
    m_timerNoResponse.setSingleShot(true);

    connect(&m_timerConnectToServer,&QTimer::timeout, this, &TcpClient02::connectToServer);
    connect(&m_timerStartSending,   &QTimer::timeout, this, &TcpClient02::sendTCPStart);
    connect(&m_timerNoResponse,     &QTimer::timeout, this, &TcpClient02::noResponseReceived);

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

    bool result1 = m_tcpClient1.connectToServer(m_servers[0].m_ipAddress, m_servers[0].m_port);
    if (!result1)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";
    bool result2 = m_tcpClient2.connectToServer(m_servers[1].m_ipAddress, m_servers[1].m_port);
    if (!result2)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";

    if (result1 && result2)
        sendTCPStart();
}

void TcpClient02::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
}

int selectNextLoop(int loop)
{
    switch (loop)
    {
    case -10: return 0;
    case 0: return 1;
    case 1: return 2;
    case 2: return 10;

    case 10: return 12;     // Skip 11
    case 11: return 12;
    case 12: return 13;
    case 13: return 14;
    case 14: return 15;
    case 15: return 16;
    case 16: return 20;

    case 20: return 21;
    case 21: return 22;
    case 22: return 30;

    case 30: return 32;     // Skip 31
    case 31: return 32;
    case 32: return 33;
    case 33: return 34;
    case 34: return 35;
    case 35: return 36;
    case 36: return 41;

    case 41: return 42;
    case 42: return 43;
    case 43: return 44;
    case 44: return 51;

    case 51: return 52;
    case 52: return 53;
    case 53: return 54;
    case 54: return 55;
    case 55: return 56;
    case 56: return 57;
    case 57: return 61;

    case 61: return 62;
    case 62: return 63;
    case 63: return 64;
    case 64: return 65;
    case 65: return 66;
    case 66: return 67;
    case 67: return -1;

    default: return -1;
    }
    return -1;
}

/**
 * @brief TcpClient02::sendTCPStart
 */
void TcpClient02::sendTCPStart()
{
    m_selection++;
    qInfo() << Q_FUNC_INFO << m_loop << m_selection;

    m_counter = 0;

    if (!configuration.m_useCoroutines)
    {
        if (m_selection < nr_message_lengths)
        {
            m_start = chrono::high_resolution_clock::now();
            QByteArray data = prepareMessage(m_selection);
            m_tcpClient1.sendMessage(data);
            m_timerNoResponse.start(10 * data.length());
            //m_tcpClient2.sendMessage(data);
        }   
    }
    else
    {
        if (m_selection >= nr_message_lengths)
        {
           m_selection = 0;
           m_loop = selectNextLoop(m_loop);
           qDebug() << "next loop = " << m_loop;
           if (m_loop == -1)
               return;
        }

        switch (m_loop)
        {
        case 0: { async_task<int> si = measurementLoop0(); } break;
        case 1: { async_task<int> si = measurementLoop1(); } break;
        case 2: { async_task<int> si = measurementLoop2(); } break;

        case 10: { async_task<int> si = measurementLoop10(); } break;
        case 11: { async_task<int> si = measurementLoop11(); } break;
        case 12: { async_task<int> si = measurementLoop12(); } break;
        case 13: { async_task<int> si = measurementLoop13(); } break;
        case 14: { async_task<int> si = measurementLoop14(); } break;
        case 15: { async_task<int> si = measurementLoop15(); } break;
        case 16: { async_task<int> si = measurementLoop16(); } break;

        case 20: { async_task<int> si = measurementLoop20(); } break;
        case 21: { async_task<int> si = measurementLoop21(); } break;
        case 22: { async_task<int> si = measurementLoop22(); } break;

        case 30: { async_task<int> si = measurementLoop30(); } break;
        case 31: { async_task<int> si = measurementLoop31(); } break;
        case 32: { async_task<int> si = measurementLoop32(); } break;
        case 33: { async_task<int> si = measurementLoop33(); } break;
        case 34: { async_task<int> si = measurementLoop34(); } break;
        case 35: { async_task<int> si = measurementLoop35(); } break;
        case 36: { async_task<int> si = measurementLoop36(); } break;

        case 41: { async_task<int> si = measurementLoop41(); } break;
        case 42: { async_task<int> si = measurementLoop42(); } break;
        case 43: { async_task<int> si = measurementLoop43(); } break;
        case 44: { async_task<int> si = measurementLoop44(); } break;

        case 51: { async_task<int> si = measurementLoop51(); } break;
        case 52: { async_task<int> si = measurementLoop52(); } break;
        case 53: { async_task<int> si = measurementLoop53(); } break;
        case 54: { async_task<int> si = measurementLoop54(); } break;
        case 55: { async_task<int> si = measurementLoop55(); } break;
        case 56: { async_task<int> si = measurementLoop56(); } break;
        case 57: { async_task<int> si = measurementLoop57(); } break;

        case 61: { async_task<int> si = measurementLoop61(); } break;
        case 62: { async_task<int> si = measurementLoop62(); } break;
        case 63: { async_task<int> si = measurementLoop63(); } break;
        case 64: { async_task<int> si = measurementLoop64(); } break;
        case 65: { async_task<int> si = measurementLoop65(); } break;
        case 66: { async_task<int> si = measurementLoop66(); } break;
        case 67: { async_task<int> si = measurementLoop67(); } break;

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
QByteArray TcpClient02::prepareMessage(int selection, int repetition)
{
    qInfo() << Q_FUNC_INFO << "selection = " << selection;
    QByteArray data = composeMessage(selection, configuration.m_step, repetition);

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
 * @param start
 * @param messageLength
 */
void TcpClient02::calculateElapsedTime(std::chrono::high_resolution_clock::time_point start,
                                       int messageLength)
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
    cout << "Time taken by 1 transaction (length = " << messageLength << ") (averaged over "
         << configuration.m_numberTransactions
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

        if (configuration.m_useCoroutines)
        {
            m_timerNoResponse.stop();
        }
        else
        {
            qInfo() << "counter:" << m_counter << "received message:" << m_message.content();
            if (++m_counter < configuration.m_numberTransactions)
            {
                QByteArray data2 = prepareMessage(m_selection);
                qInfo() << "sending message:" << data2;
                m_tcpClient1.sendMessage(data2);
                m_timerNoResponse.start(10 * data2.length());
            }
            else
            {
                calculateElapsedTime(m_start, m_message.length());
                m_timerStartSending.start(100);
            }
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);
        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        qInfo() << dataOut1.length() << ":" << dataOut1;
    }
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);

        QByteArray data2 = prepareMessage(m_selection);
        msgLength = data2.length();
        m_tcpClient1.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        async_operation<QByteArray> op2 = m_tcpClient1.start_reading();
        QByteArray dataOut2 = co_await op2;       // Hangs here because both responses were received before

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);

        QByteArray data2 = prepareMessage(m_selection);
        msgLength = data2.length();
        m_tcpClient1.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient1.start_reading();

        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);
        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        QByteArray data2 = prepareMessage(m_selection);
        m_tcpClient2.sendMessage(data2);
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop11
 * It is not a good idea to start reading after sending the message.
 * The reply may have arrived before the completion function was registered.
 * @return
 */
async_task<int> TcpClient02::measurementLoop11()
{
    int msgLength = 0;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
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
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
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
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
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
    calculateElapsedTime(start, msgLength);
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
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
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
    calculateElapsedTime(start, msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop15
 * @return
 */
async_task<int> TcpClient02::measurementLoop15()
{
    int msgLength = 0;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
        msgLength = data1.length();

        m_tcpClient1.sendMessage(data1);
        m_tcpClient2.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

        QByteArray dataOut1 = co_await op2;
        QByteArray dataOut2 = co_await op1;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop16
 * @return
 */
async_task<int> TcpClient02::measurementLoop16()
{
    int msgLength = 0;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        QByteArray data2 = prepareMessage(m_selection);
        msgLength = data1.length();

        m_tcpClient1.sendMessage(data1);
        m_tcpClient2.sendMessage(data2);

        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

        wait_all_awaitable<async_operation<QByteArray>> wa( { &op1, &op2 } );
        co_await wa;

        QByteArray dataOut1 = op1.get_result();
        QByteArray dataOut2 = op2.get_result();

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop20
 * @return
 */
async_task<int> TcpClient02::measurementLoop20()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            msgLength = data1.length();
            m_tcpClient1.sendMessage(data1);
            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            QByteArray dataOut1 = co_await op1;

            qInfo() << dataOut1.length() << ":" << dataOut1;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop21
 * @return
 */
async_task<int> TcpClient02::measurementLoop21()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            msgLength = data1.length();
            m_tcpClient1.sendMessage(data1);

            QByteArray data2 = prepareMessage(selection);
            msgLength = data2.length();
            m_tcpClient1.sendMessage(data2);

            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            QByteArray dataOut1 = co_await op1;

            async_operation<QByteArray> op2 = m_tcpClient1.start_reading();
            QByteArray dataOut2 = co_await op2;       // Hangs here because both responses were received before

            qInfo() << dataOut1.length() << ":" << dataOut1;
            qInfo() << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop22
 * @return
 */
async_task<int> TcpClient02::measurementLoop22()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            msgLength = data1.length();
            m_tcpClient1.sendMessage(data1);

            QByteArray data2 = prepareMessage(selection);
            msgLength = data2.length();
            m_tcpClient1.sendMessage(data2);

            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            async_operation<QByteArray> op2 = m_tcpClient1.start_reading();

            QByteArray dataOut1 = co_await op1;
            QByteArray dataOut2 = co_await op2;

            qInfo() << i << dataOut1.length() << ":" << dataOut1;
            qInfo() << i << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop30
 * @return
 */
async_task<int> TcpClient02::measurementLoop30()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            msgLength = data1.length();
            m_tcpClient1.sendMessage(data1);
            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            QByteArray dataOut1 = co_await op1;

            QByteArray data2 = prepareMessage(selection);
            m_tcpClient2.sendMessage(data2);
            async_operation<QByteArray> op2 = m_tcpClient2.start_reading();
            QByteArray dataOut2 = co_await op2;

            qInfo() << dataOut1.length() << ":" << dataOut1;
            qInfo() << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop31
 * @return
 */
async_task<int> TcpClient02::measurementLoop31()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
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
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop32
 * @return
 */
async_task<int> TcpClient02::measurementLoop32()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
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
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop33
 * @return
 */
async_task<int> TcpClient02::measurementLoop33()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
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
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop34
 * @return
 */
async_task<int> TcpClient02::measurementLoop34()
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
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
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop35
 * @return
 */
async_task<int> TcpClient02::measurementLoop35()
{
    int msgLength = 0;
    qDebug() << Q_FUNC_INFO << "begin";
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
            msgLength = data1.length();

            m_tcpClient1.sendMessage(data1);
            m_tcpClient2.sendMessage(data2);

            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

            QByteArray dataOut2 = co_await op2;
            QByteArray dataOut1 = co_await op1;

            qInfo() << i << dataOut1.length() << ":" << dataOut1;
            qInfo() << i << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop36
 * @return
 */
async_task<int> TcpClient02::measurementLoop36()
{
    int msgLength = 0;
    qDebug() << Q_FUNC_INFO << "begin";
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data1 = prepareMessage(selection);
            QByteArray data2 = prepareMessage(selection);
            msgLength = data1.length();

            m_tcpClient1.sendMessage(data1);
            m_tcpClient2.sendMessage(data2);

            async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
            async_operation<QByteArray> op2 = m_tcpClient2.start_reading();

            wait_all_awaitable<async_operation<QByteArray>> wa( { &op1, &op2 } );
            co_await wa;

            QByteArray dataOut1 = op1.get_result();
            QByteArray dataOut2 = op2.get_result();

            qInfo() << i << dataOut1.length() << ":" << dataOut1;
            qInfo() << i << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop40
 * @param tcpClient
 * @return
 */
async_task<int> TcpClient02::measurementLoop40(TcpClientCo& tcpClient)
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data = prepareMessage(selection);
            msgLength = data.length();
            tcpClient.sendMessage(data);
            async_operation<QByteArray> op = tcpClient.start_reading();
            QByteArray dataOut = co_await op;

            qInfo() << dataOut.length() << ":" << dataOut;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop41
 * @return
 */
async_task<int> TcpClient02::measurementLoop41()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop40(m_tcpClient1);
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop42
 * @return
 */
async_task<int> TcpClient02::measurementLoop42()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop40(m_tcpClient1);
    async_task<int> t2 = measurementLoop40(m_tcpClient2);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop43
 * @return
 */
async_task<int> TcpClient02::measurementLoop43()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop40(m_tcpClient1);
    async_task<int> t2 = measurementLoop40(m_tcpClient2);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop44
 * @return
 */
async_task<int> TcpClient02::measurementLoop44()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop40(m_tcpClient1);
    async_task<int> t2 = measurementLoop40(m_tcpClient2);
    wait_all_awaitable< async_task<int> > wa({ & t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop50
 * @param tcpClient
 * @return
 */
async_task<int> TcpClient02::measurementLoop50(TcpClientCo& tcpClient, int nrRepetitions)
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data = prepareMessage(selection, nrRepetitions);
            msgLength = data.length();
            tcpClient.sendMessage(data);

            for (int i = 0; i < nrRepetitions; i++)
            {
                async_operation<QByteArray> op = tcpClient.start_reading();
                QByteArray dataOut = co_await op;

                qInfo() << dataOut.length() << ":" << dataOut;
            }
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop51
 * @return
 */
async_task<int> TcpClient02::measurementLoop51()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop52
 * @return
 */
async_task<int> TcpClient02::measurementLoop52()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 5);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop53
 * @return
 */
async_task<int> TcpClient02::measurementLoop53()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 5);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop54
 * @return
 */
async_task<int> TcpClient02::measurementLoop54()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 5);
    wait_all_awaitable< async_task<int> > wa({ & t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop55
 * @return
 */
async_task<int> TcpClient02::measurementLoop55()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 7);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop56
 * @return
 */
async_task<int> TcpClient02::measurementLoop56()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 7);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop57
 * @return
 */
async_task<int> TcpClient02::measurementLoop57()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop50(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop50(m_tcpClient2, 7);
    wait_all_awaitable< async_task<int> > wa({ & t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop60
 * @param tcpClient
 * @return
 */
async_task<int> TcpClient02::measurementLoop60(TcpClientCo& tcpClient, int nrRepetitions)
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QByteArray data = prepareMessage(selection, nrRepetitions);
            msgLength = data.length();
            tcpClient.sendMessage(data);

            async_operation<QByteArray> op = tcpClient.start_reading(false);
            for (int i = 0; i < nrRepetitions; i++)
            {
                QByteArray dataOut = co_await op;
                op.reset();
                qInfo() << dataOut.length() << ":" << dataOut;
            }
            int idx = op.get_index();
            tcpClient.stop_reading(idx);
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop61
 * @return
 */
async_task<int> TcpClient02::measurementLoop61()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop62
 * @return
 */
async_task<int> TcpClient02::measurementLoop62()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 5);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop63
 * @return
 */
async_task<int> TcpClient02::measurementLoop63()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 5);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop64
 * @return
 */
async_task<int> TcpClient02::measurementLoop64()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 5);
    wait_all_awaitable< async_task<int> > wa({ & t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop65
 * @return
 */
async_task<int> TcpClient02::measurementLoop65()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 7);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop66
 * @return
 */
async_task<int> TcpClient02::measurementLoop66()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 7);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop67
 * @return
 */
async_task<int> TcpClient02::measurementLoop67()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop60(m_tcpClient1, 5);
    async_task<int> t2 = measurementLoop60(m_tcpClient2, 7);
    wait_all_awaitable< async_task<int> > wa({ & t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    m_timerStartSending.start(100);
    m_selection = nr_message_lengths;
    co_return 0;
}
