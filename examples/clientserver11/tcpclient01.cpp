/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include "tcpclient01.h"
#include "tcpconfig.h"

#include "wait_all_awaitable.h"

#include <chrono>
#include <iostream>
#include <iomanip>      // setprecision
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
    , m_selection(0)
    , m_loop(configuration.m_selectMeasurementLoop)

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

    connect(&m_timerConnectToServer,&QTimer::timeout, this, &TcpClient01::connectToServer);
    connect(&m_timerStartSending,   &QTimer::timeout, this, &TcpClient01::sendTCPStart);
    connect(&m_timerNoResponse,     &QTimer::timeout, this, &TcpClient01::noResponseReceived);
    connect(&m_tcpClient,           &TcpClient::connectedSig, this, &TcpClient01::connected);

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
 * @brief TcpClient01::start
 */
void TcpClient01::start()
{
    qInfo() << Q_FUNC_INFO;
    connectToServerDelayed();
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
 * @brief TcpClient01::connectToServerDelayed
 */
void TcpClient01::connectToServerDelayed()
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
 * @brief TcpClient01::connectToServer
 */
void TcpClient01::connectToServer()
{
    qInfo() << "";
    qInfo() << Q_FUNC_INFO;

    qInfo() << "m_server.m_ipAddress = " << m_server.m_ipAddress << ", m_server.m_port = " << m_server.m_port;

    bool result = m_tcpClient.connectToServer(m_server.m_ipAddress, m_server.m_port);
    if (!result)
        qDebug() << Q_FUNC_INFO << "immediate connection failed";
}

/**
 * @brief TcpClient01::connected
 */
void TcpClient01::connected()
{
    qDebug() << Q_FUNC_INFO;
    sendTCPStart();
}

/**
 * @brief TcpClient01::noResponseReceived
 */
void TcpClient01::noResponseReceived()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief selectNextLoop
 * @param loop
 * @return
 */
int selectNextLoop(int loop)
{
    switch (loop)
    {
    case -10: return 0;
    case 0: return 1;
    case 1: return 2;
    case 2: return 3;
    case 3: return 4;
    case 4: return -1;

    default: return -1;
    }
    return -1;
}
/**
 * @brief TcpClient01::sendTCPStart
 */
void TcpClient01::sendTCPStart()
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
            m_tcpClient.sendMessage(data);
            m_timerNoResponse.start(10 * data.length());
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
        case 3: { async_task<int> si = measurementLoop3(); } break;
        case 4: { async_task<int> si = measurementLoop4(); } break;

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

        if (configuration.m_useCoroutines)
        {
            QByteArray msg = m_message.content();
            emit responseReceivedSig(msg);
            m_timerNoResponse.stop();
        }
        else
        {
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
        }
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
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI2, "%p: TcpClient01::start_reading(): index = %d\n", this, index);
    assert(m_async_operations[index] == nullptr);
    async_operation<QByteArray> ret{ this, index };
    start_read(index);
    return ret;
}

/**
 * @brief TcpClient01::start_read
 * @param idx
 */
void TcpClient01::start_read(const int idx)
{
    print(PRI2, "%p: TcpClient01::start_read(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(this, &TcpClient01::responseReceivedSig,
        [this, idx](QByteArray msg)
        {
            //qDebug() << msg.length() << ":" << msg;

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
                print(PRI2, "%p: TcpClient01::start_read(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }

            if (!disconnect(m_connections[idx]))
            {
                print(PRI1, "%p: TcpClient01::start_read(): idx = %d, Warning: disconnect failed\n", this, idx);
            }
        }
    );
}

/**
 * @brief TcpClient01::measurementLoop0
 * @return
 */
async_task<int> TcpClient01::measurementLoop0()
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data1);
        m_timerNoResponse.start(10 * data1.length());
        async_operation<QByteArray> op = start_reading();
        QByteArray dataOut = co_await op;
        qInfo() << dataOut.length() << ":" << dataOut;
    }
    calculateElapsedTime(start);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop1
 * @return
 */
async_task<int> TcpClient01::measurementLoop1()
{
    //qDebug() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        async_operation<QByteArray> op = start_reading();
        QByteArray data1 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data1);
        m_timerNoResponse.start(10 * data1.length());
        QByteArray dataOut = co_await op;
        qInfo() << dataOut.length() << ":" << dataOut;
    }
    calculateElapsedTime(start);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop2
 * @return
 */
async_task<int> TcpClient01::measurementLoop2()
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        QByteArray dataOut1 = co_await op1;
        async_operation<QByteArray> op2 = start_reading();
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop3
 * @return
 */
async_task<int> TcpClient01::measurementLoop3()
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        async_operation<QByteArray> op2 = start_reading();
        QByteArray dataOut1 = co_await op1;
        QByteArray dataOut2 = co_await op2;

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    m_timerStartSending.start(100);
    co_return 0;
}

/**
 * @brief TcpClient01::measurementLoop4
 * @return
 */
async_task<int> TcpClient01::measurementLoop4()
{
    qInfo() << Q_FUNC_INFO;
    std::chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < configuration.m_numberTransactions; i++)
    {
        QByteArray data1 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data1);
        QByteArray data2 = prepareMessage(m_selection);
        m_tcpClient.sendMessage(data2);

        async_operation<QByteArray> op1 = start_reading();
        async_operation<QByteArray> op2 = start_reading();

        wait_all_awaitable<async_operation<QByteArray>> wa( { &op1, &op2 } );
        co_await wa;

        QByteArray dataOut1 = op1.get_result();
        QByteArray dataOut2 = op2.get_result();

        qInfo() << dataOut1.length() << ":" << dataOut1;
        qInfo() << dataOut2.length() << ":" << dataOut2;
    }
    calculateElapsedTime(start);
    m_timerStartSending.start(100);
    co_return 0;
}
