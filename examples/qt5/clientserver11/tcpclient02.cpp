/**
 * @file tcpclient02.cpp
 * @brief
 * Implementation of the second TCP client application
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QCoreApplication>

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision

#include "tcpclient02.h"
#include "tcpconfig.h"

#include <corolib/when_all.h>
using namespace std;

static int nr_message_lengths = 10;

/**
 * @brief TcpClient02::TcpClient02
 * @param parent
 */
TcpClient02::TcpClient02(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_errorCounter(0)
    , m_timerNoResponse(this)

    , m_tcpClient1(configuration.m_selectImplementation,
                   "client1",
                   true,
                   configuration.m_waitForConnectionTimeout,
                   configuration.m_reconnectTimeout,
                   configuration.m_reconnectTimeoutAfterDisconnect)

    , m_tcpClient2(configuration.m_selectImplementation,
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

    m_timerNoResponse.setSingleShot(true);

    connect(&m_timerNoResponse, &QTimer::timeout, this, &TcpClient02::noResponseReceived);

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
 * @brief TcpClient02::connectToServerAsync
 */
async_task<int> TcpClient02::connectToServerAsync()
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    async_operation<void> c1 = m_tcpClient1.start_connecting(m_servers[0].m_ipAddress, m_servers[0].m_port);
    async_operation<void> c2 = m_tcpClient2.start_connecting(m_servers[1].m_ipAddress, m_servers[1].m_port);
    when_all wa({ &c1, &c2 });

    qDebug() << Q_FUNC_INFO << "before co_await wa;";
    co_await wa;
    qDebug() << Q_FUNC_INFO << "after co_await wa;";

    co_return 0;
}

/**
 * @brief TcpClient02::noResponseReceived
 */
void TcpClient02::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
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
        m_timerNoResponse.stop();
    } // while
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
 * @brief TcpClient02::measurementLoop0 prepares a message,
 * sends this message to the first server,
 * starts reading the response
 * and co_awaits the response.
 * It repeats these actions configuration.m_numberTransactions times.
 * @note It is not a good idea to start reading after sending the message.
 * The response may have arrived before the completion function was registered.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop0(int selection)
{
    int msgLength = 0;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        qInfo() << Q_FUNC_INFO << i;
#if 0
        QTimer timer(this);
        timer.setSingleShot(true);
        async_operation<void> opT = m_tcpClient1.start_timer(timer, 100);
        qInfo() << Q_FUNC_INFO << "before co_await opT";
        co_await opT;
        qInfo() << Q_FUNC_INFO << "after co_await opT";
#endif
        QByteArray data1 = prepareMessage(selection);
        msgLength = data1.length();
        m_tcpClient1.sendMessage(data1);
        async_operation<QByteArray> op1 = m_tcpClient1.start_reading();
        QByteArray dataOut1 = co_await op1;

        qInfo() << dataOut1.length() << ":" << dataOut1;
    }
    calculateElapsedTime(start, msgLength);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop1 prepares a first message,
 * sends this message to the first server,
 * prepares a second message,
 * sends this message again to the first server,
 * starts reading the response on the first message,
 * co_awaits the response,
 * starts reading the response on the second message
 * and co_awaits the response.
 * @note It is not a good idea to start reading after sending the message.
 * The response may have arrived before the completion function was registered.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop1(int selection)
{
    int msgLength = 0;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        qInfo() << Q_FUNC_INFO << i;

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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop2
 * @note It is not a good idea to start reading after sending the message.
 * The response may have arrived before the completion function was registered.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop2(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop10
 * @note It is not a good idea to start reading after sending the message.
 * The response may have arrived before the completion function was registered.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop10(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop11
 * @note It is not a good idea to start reading after sending the message.
 * The response may have arrived before the completion function was registered.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop11(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop12 prepares two messages,
 * starts reading the response from the first server, 
 * sends the first message to the first server,
 * co_awaits the response from the first server,
 * starts reading the response from the second server, 
 * sends the second message to the second server
 * and co_awaits the response from the second server.
 * It repeats these actions configuration.m_numberTransactions times.
 * @note By start reading the response before the message has been sent,
 * the completion handler is surely in place at the moment the response arrives.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop12(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop13 is a variant of measurementLoop12
 * with re-ordering of some statements.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop13(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop14 is a variant of measurementLoop13.
 * measurementLoop14 first sends a message to the two servers before it starts reading
 * the responses.
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop14(int selection)
{
    int msgLength = 0;
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
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop15
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop15(int selection)
{
    int msgLength = 0;
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

        QByteArray dataOut1 = co_await op2;
        QByteArray dataOut2 = co_await op1;

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop16
 * @param selection
 * @return
 */
async_task<int> TcpClient02::measurementLoop16(int selection)
{
    int msgLength = 0;
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

        when_all wa( { &op1, &op2 } );
        co_await wa;

        QByteArray dataOut1 = op1.get_result();
        QByteArray dataOut2 = op2.get_result();

        qInfo() << i << dataOut1.length() << ":" << dataOut1;
        qInfo() << i << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start, msgLength);
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop20 (and the following functions) use a double for loop.
 * The outer loop iterates over a number of message lengths to be generated, 
 * the inner loop sends a message of a given length a number of times to the server and reads the response of the server.
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

            when_all wa( { &op1, &op2 } );
            co_await wa;

            QByteArray dataOut1 = op1.get_result();
            QByteArray dataOut2 = op2.get_result();

            qInfo() << i << dataOut1.length() << ":" << dataOut1;
            qInfo() << i << dataOut2.length() << ":" << dataOut2;
        }
        calculateElapsedTime(start, msgLength);
    }
    qDebug() << Q_FUNC_INFO << "end";
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
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
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
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
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
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
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
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
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
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop70
 * @param tcpClient
 * @return
 */
async_task<int> TcpClient02::measurementLoop70(TcpClientCo& tcpClient, int nrRepetitions, int timeout)
{
    qDebug() << Q_FUNC_INFO << "begin";
    int msgLength = 0;
    for (int selection = 0; selection < nr_message_lengths; selection++)
    {
        std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
        for (int i = 0; i < configuration.m_numberTransactions; i++)
        {
            QTimer timer(this);
            timer.setSingleShot(true);
            qInfo() << Q_FUNC_INFO << "async_operation<void> opT = tcpClient.start_timer(timer, timeout)";
            async_operation<void> opT = tcpClient.start_timer(timer, timeout);
            qInfo() << Q_FUNC_INFO << "before co_await opT";
            co_await opT;
            qInfo() << Q_FUNC_INFO << "after co_await opT";

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
 * @brief TcpClient02::measurementLoop71
 * @return
 */
async_task<int> TcpClient02::measurementLoop71()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 50);
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop72
 * @return
 */
async_task<int> TcpClient02::measurementLoop72()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 5, 20);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop73
 * @return
 */
async_task<int> TcpClient02::measurementLoop73()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 5, 20);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop74
 * @return
 */
async_task<int> TcpClient02::measurementLoop74()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 5, 20);
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop75
 * @return
 */
async_task<int> TcpClient02::measurementLoop75()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 7, 20);
    co_await t1;
    co_await t2;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop76
 * @return
 */
async_task<int> TcpClient02::measurementLoop76()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 7, 20);
    co_await t2;
    co_await t1;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::measurementLoop77
 * @return
 */
async_task<int> TcpClient02::measurementLoop77()
{
    qDebug() << Q_FUNC_INFO << "begin";
    async_task<int> t1 = measurementLoop70(m_tcpClient1, 5, 30);
    async_task<int> t2 = measurementLoop70(m_tcpClient2, 7, 20);
    when_all wa({ &t1, &t2 });
    co_await wa;
    qDebug() << Q_FUNC_INFO << "end";
    co_return 0;
}

/**
 * @brief TcpClient02::mainTask
 * @return
 */
async_task<int> TcpClient02::mainTask()
{
    qInfo() << Q_FUNC_INFO;
    int selection = 0;
    print(PRI1, "--- mainTask: co_await connectToServerAsync();\n");
    co_await connectToServerAsync();

    print(PRI1, "--- mainTask: measurementLoop0(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop0(selection);
    print(PRI1, "--- mainTask: measurementLoop1(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop1(selection);
    print(PRI1, "--- mainTask: measurementLoop2(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop2(selection);

    print(PRI1, "--- mainTask: measurementLoop10(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop10(selection);
#if 0
    print(PRI1, "--- mainTask: measurementLoop11(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop11(selection);
#endif
    print(PRI1, "--- mainTask: measurementLoop12(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop12(selection);
#if 0
    print(PRI1, "--- mainTask: measurementLoop13(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop13(selection);
    print(PRI1, "--- mainTask: measurementLoop14(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop14(selection);
    print(PRI1, "--- mainTask: measurementLoop15(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop15(selection);
    print(PRI1, "--- mainTask: measurementLoop16(selection);\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop16(selection);
#endif
    print(PRI1, "--- mainTask: measurementLoop20();\n");
    co_await measurementLoop20();
    print(PRI1, "--- mainTask: measurementLoop21();\n");
    co_await measurementLoop21();
#if 0
    print(PRI1, "--- mainTask: measurementLoop22();\n");
    co_await measurementLoop22();
#endif
    print(PRI1, "--- mainTask: measurementLoop30();\n");
    co_await measurementLoop30();
#if 0
    print(PRI1, "--- mainTask: measurementLoop31();\n");
    co_await measurementLoop31();
#endif
    print(PRI1, "--- mainTask: measurementLoop32();\n");
    co_await measurementLoop32();
#if 0
    print(PRI1, "--- mainTask: measurementLoop33();\n");
    co_await measurementLoop33();
    print(PRI1, "--- mainTask: measurementLoop34();\n");
    co_await measurementLoop34();
    print(PRI1, "--- mainTask: measurementLoop35();\n");
    co_await measurementLoop35();
    print(PRI1, "--- mainTask: measurementLoop36();\n");
    co_await measurementLoop36();
#endif
    print(PRI1, "--- mainTask: measurementLoop41();\n");
    co_await measurementLoop41();
    print(PRI1, "--- mainTask: measurementLoop42();\n");
    co_await measurementLoop42();
    print(PRI1, "--- mainTask: measurementLoop43();\n");
    co_await measurementLoop43();
#if 0
    print(PRI1, "--- mainTask: measurementLoop44();\n");
    co_await measurementLoop44();
#endif
    print(PRI1, "--- mainTask: measurementLoop51();\n");
    co_await measurementLoop51();
    print(PRI1, "--- mainTask: measurementLoop52();\n");
    co_await measurementLoop52();
    print(PRI1, "--- mainTask: measurementLoop53();\n");
    co_await measurementLoop53();
    print(PRI1, "--- mainTask: measurementLoop54();\n");
    co_await measurementLoop54();
    print(PRI1, "--- mainTask: measurementLoop55();\n");
    co_await measurementLoop55();
    print(PRI1, "--- mainTask: measurementLoop56();\n");
    co_await measurementLoop56();
    print(PRI1, "--- mainTask: measurementLoop57();\n");
    co_await measurementLoop57();

    print(PRI1, "--- mainTask: measurementLoop61();\n");
    co_await measurementLoop61();
    print(PRI1, "--- mainTask: measurementLoop62();\n");
    co_await measurementLoop62();
    print(PRI1, "--- mainTask: measurementLoop63();\n");
    co_await measurementLoop63();
    print(PRI1, "--- mainTask: measurementLoop64();\n");
    co_await measurementLoop64();
    print(PRI1, "--- mainTask: measurementLoop65();\n");
    co_await measurementLoop65();
    print(PRI1, "--- mainTask: measurementLoop66();\n");
    co_await measurementLoop66();
    print(PRI1, "--- mainTask: measurementLoop67();\n");
    co_await measurementLoop67();

    print(PRI1, "--- mainTask: measurementLoop71();\n");
    co_await measurementLoop71();
    print(PRI1, "--- mainTask: measurementLoop72();\n");
    co_await measurementLoop72();
    print(PRI1, "--- mainTask: measurementLoop73();\n");
    co_await measurementLoop73();
    print(PRI1, "--- mainTask: measurementLoop74();\n");
    co_await measurementLoop74();
    print(PRI1, "--- mainTask: measurementLoop75();\n");
    co_await measurementLoop75();
    print(PRI1, "--- mainTask: measurementLoop76();\n");
    co_await measurementLoop76();
    print(PRI1, "--- mainTask: measurementLoop77();\n");
    co_await measurementLoop77();

    QCoreApplication::quit();

    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 0;
}
