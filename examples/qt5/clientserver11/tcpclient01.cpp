/**
 * @file tcpclient01.cpp
 * @brief 
 * Implementation of the first TCP client appliation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QCoreApplication>

#include "tcpclient01.h"
#include "tcpconfig.h"

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision

#include <corolib/when_all.h>

using namespace std;

static int nr_message_lengths = 10;

/**
 * @brief TcpClient01::TcpClient01
 * @param parent
 */
TcpClient01::TcpClient01(QObject *parent, MessageCheck check)
    : QObject(parent)
    , m_server(configuration.m_server)

    , m_errorCounter(0)

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

    connect(&m_timerNoResponse,     &QTimer::timeout, this, &TcpClient01::noResponseReceived);

    nr_message_lengths = configuration.m_numberMessages;

    configureTCP();
}

/**
 * @brief TcpClient01::configureTCP
 */
void TcpClient01::configureTCP()
{
    qInfo() << Q_FUNC_INFO;

    m_tcpClient.configure();
    connect(&m_tcpClient, &TcpClient::readyReadTcpSig, this, &TcpClient01::readyReadTcp);
}

/**
 * @brief TcpClient01::addErrorMessage
 * @param message
 */
void TcpClient01::addErrorMessage(const QString &message)
{
    qDebug() << "ERROR:" << message;
}

/**
 * @brief TcpClient01::addWarningMessage
 * @param message
 */
void TcpClient01::addWarningMessage(const QString &message)
{
    qDebug() << "WARNING" << message;
}

/**
 * @brief TcpClient01::addInfoMessage
 * @param message
 */
void TcpClient01::addInfoMessage(const QString &message)
{
    qDebug() << "INFO:" << message;
}

/**
 * @brief TcpClient01::noResponseReceived
 */
void TcpClient01::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient02::prepareMessage
 * @param selection
 * @return
 */
QByteArray TcpClient01::prepareMessage(int selection)
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
 * @brief TcpClient01::quit
 */
void TcpClient01::quit()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpClient01::acceptError
 * @param socketError
 */
void TcpClient01::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << "acceptError: " << socketError;
}

/**
 * @brief TcpClient01::calculateElapsedTime
 */
void TcpClient01::calculateElapsedTime(std::chrono::high_resolution_clock::time_point start)
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
 * @brief TcpClient01::readyReadTcp
 * @param data
 */
void TcpClient01::readyReadTcp(QByteArray& data)
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
        m_timerNoResponse.stop();

    } // while
}

/**
 * @brief TcpClient01::errorOccurred
 * @param socketError
 */
void TcpClient01::errorOccurred(QAbstractSocket::SocketError socketError)
{
    qDebug() << Q_FUNC_INFO << socketError;
}

/**
 * @brief TcpClient01::hostFound
 */
void TcpClient01::hostFound()
{

}

/**
 * @brief TcpClient01::stateChanged
 * @param socketState
 */
void TcpClient01::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << "Socket changed to: " << socketState;
}

// Coroutine related
// =================

/**
 * @brief TcpClient01::start_reading
 * @return
 */
async_operation<QByteArray> TcpClient01::start_reading()
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClient01::start_reading(): index = %d\n", this, index);
    async_operation<QByteArray> ret{ this, index };
    start_reading_impl(index);
    return ret;
}

/**
 * @brief TcpClient01::start_reading_impl uses Qt's connect to associate
 * the TcpClient01::responseReceivedSig signal emitted by itself
 * with a lambda used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal at the end of its invocation.
 * @param idx
 */
void TcpClient01::start_reading_impl(const int idx)
{
    print(PRI2, "%p: TcpClient01::start_reading_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(this, &TcpClient01::responseReceivedSig,
        [this, idx](QByteArray msg)
        {
            async_operation_base* om_async_operation = m_async_operations[idx];
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
                print(PRI2, "%p: TcpClient01::start_reading_impl(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }

            if (!disconnect(m_connections[idx]))
            {
                print(PRI1, "%p: TcpClient01::start_reading_impl(): idx = %d, Warning: disconnect failed\n", this, idx);
            }
        }
    );
}

/**
 * @brief TcpClient01::start_timer
 * @param timer
 * @param ms
 * @return
 */
async_operation<void> TcpClient01::start_timer(QTimer& timer, int ms)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClient01::start_timer(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief TcpClient01::start_timer_impl uses Qt's connect to associate
 * the QTimer::timeout signal function emitted by tmr
 * with a lambda that is used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal
 * at the end of its invocation.
 * @param idx
 * @param tmr
 * @param ms
 */
void TcpClient01::start_timer_impl(const int idx, QTimer& tmr, int ms)
{
    print(PRI2, "%p: TcpClient01::start_timer_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    tmr.start(ms);

    m_connections[idx] = connect(&tmr, &QTimer::timeout,
         [this, idx]()
         {
             print(PRI2, "%p: TcpClient01::handle_timer() lambda: idx = %d\n", this, idx);

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
                 print(PRI2, "%p: TcpClient01::handle_timer(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
             }

             if (!disconnect(m_connections[idx]))
             {
                 print(PRI1, "%p: TcpClient::handle_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
             }
         }
         );
}

void TcpClient01::connectToServer(QString& serverIPaddress, quint16 serverPort)
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    qInfo() << "serverIPaddress = " << serverIPaddress << ", serverPort = " << serverPort;

    bool result = m_tcpClient.connectToServer(serverIPaddress, serverPort);
    if (!result)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";
}

/**
 * @brief TcpClient01::start_connecting
 * @param serverIpAddress
 * @param port
 * @return
 */
async_operation<void> TcpClient01::start_connecting(QString& serverIpAddress, quint16 port)
{
    int index = get_free_index();
    print(PRI2, "%p: TcpClient01::start_connecting(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_connecting_impl(index, serverIpAddress, port);
    return ret;
}

/**
 * @brief TcpClient01::start_connecting_impl uses Qt's connect to associate
 * the TcpClient::connectedSig signal function emitted by itself
 * with a lambda that is used as slot functor.
 * The lambda uses Qt's disconnect to break the association with the signal function.
 * @param idx
 * @param serverIpAddress
 * @param port
 */
void TcpClient01::start_connecting_impl(const int idx, QString& serverIpAddress, quint16 port)
{
    print(PRI2, "%p: TcpClient::start_connecting_impl(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(&m_tcpClient, &TcpClient::connectedSig,
         [this, idx]()
         {
             print(PRI2, "%p: TcpClient01::handle_connect() lambda: idx = %d\n", this, idx);

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
                 print(PRI2, "%p: TcpClient::handle_connect(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
             }

             if (!disconnect(m_connections[idx]))
             {
                 print(PRI1, "%p: TcpClient01::handle_connect(): idx = %d, Warning: disconnect failed\n", this, idx);
             }
         }
         );

    connectToServer(serverIpAddress, port);
}

/**
 * @brief TcpClient01::connectToServerAsync
 */
async_task<int> TcpClient01::connectToServerAsync()
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    async_operation<void> c = start_connecting(m_server.m_ipAddress, m_server.m_port);
    co_await c;

    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop0
 * @param selection
 * @return
 */
async_task<int> TcpClient01::measurementLoop0(int selection)
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(selection);
        m_tcpClient.sendMessage(data1);
        m_timerNoResponse.start(10 * data1.length());
        async_operation<QByteArray> op = start_reading();
        QByteArray dataOut = co_await op;
        qInfo() << dataOut.length() << ":" << dataOut;
    }
    calculateElapsedTime(start);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop1
 * @param selection
 * @return
 */
async_task<int> TcpClient01::measurementLoop1(int selection)
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        async_operation<QByteArray> op = start_reading();
        QByteArray data1 = prepareMessage(selection);
        m_tcpClient.sendMessage(data1);
        m_timerNoResponse.start(10 * data1.length());
        QByteArray dataOut = co_await op;
        qInfo() << dataOut.length() << ":" << dataOut;
    }
    calculateElapsedTime(start);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop2
 * @param selection
 * @return
 */
async_task<int> TcpClient01::measurementLoop2(int selection)
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        QByteArray dataOut1 = co_await op1;
        async_operation<QByteArray> op2 = start_reading();
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop3
 * @param selection
 * @return
 */
async_task<int> TcpClient01::measurementLoop3(int selection)
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        async_operation<QByteArray> op2 = start_reading();
        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop4
 * @param selection
 * @return
 */
async_task<int> TcpClient01::measurementLoop4(int selection)
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        async_operation<QByteArray> op2 = start_reading();

        when_all wa( { &op1, &op2 } );
        co_await wa;

        QByteArray dataOut1 = op1.get_result();
        QByteArray dataOut2 = op2.get_result();

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    co_return 0;
}

/**
 * @brief TcpClient01::mainTask
 * @return
 */
async_task<int> TcpClient01::mainTask()
{
    qInfo() << Q_FUNC_INFO;
    int selection = 0;

    print(PRI1, "--- mainTask: co_await connectToServerAsync();\n");
    co_await connectToServerAsync();

    print(PRI1, "--- mainTask: measurementLoop0();\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop0(selection);

    print(PRI1, "--- mainTask: measurementLoop1();\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop1(selection);

    print(PRI1, "--- mainTask: measurementLoop2();\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop2(selection);

    print(PRI1, "--- mainTask: measurementLoop3();\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop3(selection);

    print(PRI1, "--- mainTask: measurementLoop4();\n");
    for (selection = 0; selection < nr_message_lengths; ++selection)
        co_await measurementLoop4(selection);

    QCoreApplication::quit();

    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 0;
}
