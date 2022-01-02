/**
 * @file tcpserver02.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <QThread>

#include "wait_all_awaitable.h"

#include "tcpserver02.h"
#include "tcpconfig.h"

/**
 * @brief TcpServer02::TcpServer02
 * @param parent
 */
TcpServer02::TcpServer02(QObject *parent, MessageCheck check)
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
 * @brief TcpServer02::configureTCP
 */
void TcpServer02::configureTCP()
{
    qInfo() << Q_FUNC_INFO;

    m_tcpServer.configure();
    if (!configuration.m_useCoroutines)
    {
        connect(&m_tcpServer, &TcpServer::readyReadTcpSig,       this, &TcpServer02::readyReadTcp);
        connect(&m_tcpServer, &TcpServer::newTCPConnectionSig,   this, &TcpServer02::newTCPConnection);
        connect(&m_tcpServer, &TcpServer::disconnectedClientSig, this, &TcpServer02::disconnectedTCPClient);
    }
}

/**
 * @brief TcpServer02::start
 * called from main() after having created a TcpServer02 object
 *
 * Note:
 *      server.listen(QHostAddress::LocalHost, port);
 * accepts only connections from clients using "localhost" or "127.0.0.1",
 * not even if the IP address is the same as that of the computer.
 *
 */
void TcpServer02::start()
{
    qInfo() << Q_FUNC_INFO;
    m_tcpServer.startListening(m_serverPort);

    if (configuration.m_useCoroutines)
    {
        mainTask();
    }
}

/**
 * @brief TcpServer02::addErrorMessage
 * @param message
 */
void TcpServer02::addErrorMessage(const QString &message)
{
    qDebug() << "ERROR:" << message;
}

/**
 * @brief TcpServer02::addWarningMessage
 * @param message
 */
void TcpServer02::addWarningMessage(const QString &message)
{
    qDebug() << "WARNING" << message;
}

/**
 * @brief TcpServer02::addInfoMessage
 * @param message
 */
void TcpServer02::addInfoMessage(const QString &message)
{
    qDebug() << "INFO:" << message;
}


/**
 * @brief TcpServer02::quit
 */
void TcpServer02::quit()
{
    qDebug() << Q_FUNC_INFO;
    //m_tcpserver.close();      TBC
}

/**
 * @brief TcpServer02::acceptError
 * @param socketError
 */
void TcpServer02::acceptError(QAbstractSocket::SocketError socketError)
{
    qInfo() << "acceptError: " << socketError;
}


/**
 * @brief TcpServer02::readyReadTcp
 * @param data
 */
void TcpServer02::readyReadTcp(QTcpSocket* sock, QByteArray& data)
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

        QByteArray content = m_message.content();
        qInfo() << "TCPIP: " << content;
        int nrRepetitions = content[1];
        for (int i = 0; i < nrRepetitions; i++)
        {
            QThread::msleep(configuration.m_delayBeforeReply);
            m_tcpServer.sendMessage(sock, content);
        }
    } // while
}

/**
 * @brief TcpServer02::newTCPConnection
 */
void TcpServer02::newTCPConnection()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer02::disconnectedTCPClient
 */
void TcpServer02::disconnectedTCPClient()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer02::connected
 */
void TcpServer02::connected()
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief TcpServer02::errorOccurred
 * @param socketError
 */
void TcpServer02::errorOccurred(QAbstractSocket::SocketError socketError)
{
    qDebug() << Q_FUNC_INFO << socketError;
}

/**
 * @brief TcpServer02::hostFound
 */
void TcpServer02::hostFound()
{

}

/**
 * @brief TcpServer02::stateChanged
 * @param socketState
 */
void TcpServer02::stateChanged(QAbstractSocket::SocketState socketState)
{
    qInfo() << "Socket changed to: " << socketState;
}

// Coroutine related
// =================

/**
 * @brief TcpServer02::start_accepting
 * @param doDisconnect
 * @return
 */
async_operation<int> TcpServer02::start_accepting(bool doDisconnect)
{
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI2, "%p: TcpServer02::start_accepting(): index = %d\n", this, index);
    assert(m_async_operations[index] == nullptr);
    async_operation<int> ret{ this, index };
    start_accept(index, doDisconnect);
    return ret;
}

/**
 * @brief TcpServer02::start_accept
 * @param idx
 * @param doDisconnect
 */
void TcpServer02::start_accept(const int idx, bool doDisconnect)
{
    print(PRI2, "%p: TcpServer02::start_accept(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(&m_tcpServer, &TcpServer::newTCPConnectionSig,
        [this, idx, doDisconnect]()
        {
            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<int>* om_async_operation_t =
                dynamic_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->set_result(0);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpServer02::start_accept(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }
            if (doDisconnect)
            {
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: TcpServer02::start_accept(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

/**
 * @brief TcpServer02::start_reading
 * @param doDisconnect
 * @return
 */
async_operation<readInfo> TcpServer02::start_reading(bool doDisconnect)
{
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI2, "%p: TcpServer02::start_reading(): index = %d\n", this, index);
    assert(m_async_operations[index] == nullptr);
    async_operation<readInfo> ret{ this, index };
    start_read(index, doDisconnect);
    return ret;
}

/**
 * @brief TcpServer02::start_read
 * @param idx
 * @param doDisconnect
 */
void TcpServer02::start_read(const int idx, bool doDisconnect)
{
    print(PRI2, "%p: TcpServer02::start_read(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(&m_tcpServer, &TcpServer::readyReadTcpSig,
        [this, idx, doDisconnect](QTcpSocket* sock, QByteArray& data)
        {
            readInfo readInfo_;
            readInfo_.sock = sock;
            readInfo_.data = data;

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<readInfo>* om_async_operation_t =
                dynamic_cast<async_operation<readInfo>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->set_result(readInfo_);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpServer02::start_read(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }
            if (doDisconnect)
            {
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: TcpServer02::start_read(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

/**
 * @brief TcpServer02::start_timer
 * @param timer
 * @param ms
 * @param doDisconnect
 * @return
 */
async_operation<void> TcpServer02::start_timer(QTimer& timer, int ms, bool doDisconnect)
{
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI2, "%p: TcpServer02::start_timer(): index = %d\n", this, index);
    assert(m_async_operations[index] == nullptr);
#if 0
    while (m_async_operations[index] != nullptr)
    {
        print(PRI2, "%p: TcpServer02::start_timer(): (retry) index = %d\n", this, index);
        index = (index + 1) & (NROPERATIONS - 1);
    }
#endif
    async_operation<void> ret{ this, index };
    start_tmr(index, timer, ms);
    return ret;
}

/**
 * @brief TcpServer02::start_tmr
 * @param idx
 * @param tmr
 * @param ms
 * @param doDisconnect
 */
void TcpServer02::start_tmr(const int idx, QTimer& tmr, int ms, bool doDisconnect)
{
    print(PRI2, "%p: TcpServer02::start_tmr(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    tmr.start(ms);

    m_connections[idx] = connect(&tmr, &QTimer::timeout,
        [this, idx, doDisconnect]()
        {
            print(PRI2, "%p: TcpServer02::start_tmr() lambda: idx = %d\n", this, idx);

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
                print(PRI1, "%p: TcpServer02::start_tmr(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }
            if (doDisconnect)
            {
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: TcpServer02::start_tmr(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

/**
 * @brief TcpServer02::start_disconnecting
 * @param doDisconnect
 * @return
 */
async_operation<int> TcpServer02::start_disconnecting(bool doDisconnect)
{
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI2, "%p: TcpServer02::start_disconnecting(): index = %d\n", this, index);
    assert(m_async_operations[index] == nullptr);
    async_operation<int> ret{ this, index };
    start_disconnect(index, doDisconnect);
    return ret;
}

/**
 * @brief TcpServer02::start_disconnect
 * @param idx
 * @param doDisconnect
 */
void TcpServer02::start_disconnect(const int idx, bool doDisconnect)
{
    print(PRI2, "%p: TcpServer02::start_disconnect(): idx = %d, operation = %p\n", this, idx, m_async_operations[idx]);

    m_connections[idx] = connect(&m_tcpServer, &TcpServer::disconnectedClientSig,
        [this, idx, doDisconnect]()
        {
            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<int>* om_async_operation_t =
                dynamic_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->set_result(0);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: TcpServer02::start_disconnect(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }
            if (doDisconnect)
            {
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: TcpServer02::start_tmr(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

// Using coroutines
// ================

/**
 * @brief TcpServer02::acceptTask
 * @return
 */
async_task<int> TcpServer02::acceptTask()
{
    qDebug() << Q_FUNC_INFO;

    async_operation<int> op_accept = start_accepting();
    while (1)
    {
        co_await op_accept;
        op_accept.reset();
        qDebug() << Q_FUNC_INFO << "after co_await op_accept";
    }

    co_return 1;
}

/**
 * @brief TcpServer02::readTask
 * @return
 */
async_task<int> TcpServer02::readTask()
{
    qDebug() << Q_FUNC_INFO;


    static int nrMessages = 0;

    QTimer timer(this);
    timer.setSingleShot(true);
    qInfo() << Q_FUNC_INFO << "async_operation<void> opT = start_timer(timer, 0)";
    async_operation<void> op_timer = start_timer(timer, 0);
    co_await op_timer;
    op_timer.reset();

    async_operation<readInfo> op_read = start_reading();
    while (1)
    {
        readInfo readInfo_ = co_await op_read;
        op_read.reset();
        nrMessages++;
        qInfo() << Q_FUNC_INFO << "----------- after co_await op_read -----------" << nrMessages;

        QTcpSocket* sock = readInfo_.sock;
        QByteArray  data = readInfo_.data;

        qInfo() << Q_FUNC_INFO << ":" << data.length() << data;

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

            QByteArray content = m_message.content();
            qInfo() << "TCPIP: " << content;
            int nrRepetitions = content[1];

            for (int i = 0; i < nrRepetitions; i++)
            {
                qInfo() << Q_FUNC_INFO << ": i =" << i;
#if 0
                QThread::msleep(configuration.m_delayBeforeReply);
#else
                timer.start(configuration.m_delayBeforeReply);
                qInfo() << Q_FUNC_INFO << "co_await opT";
                co_await op_timer;
                op_timer.reset();
#endif
                qInfo() << Q_FUNC_INFO << "m_tcpServer.sendMessage(sock, content)";
                m_tcpServer.sendMessage(sock, content);
            } // for (int i = 0; i < nrRepetitions; i++)
        } // while (m_message.composeMessage(data, length, index))
    } // while (1)

    co_return 1;
}

/**
 * @brief TcpServer02::disconnectTask
 * @return
 */
async_task<int> TcpServer02::disconnectTask()
{
    qDebug() << Q_FUNC_INFO;

    async_operation<int> op_disconnect = start_disconnecting();
    while (1)
    {
        co_await op_disconnect;
        op_disconnect.reset();
        qDebug() << Q_FUNC_INFO << "after co_await op_disconnect";
    }

    co_return 1;
}

/**
 * @brief TcpServer02::mainTask
 * @return
 */
async_task<int> TcpServer02::mainTask()
{
    qDebug() << Q_FUNC_INFO;

    async_task<int> t1 = acceptTask();
    async_task<int> t2 = readTask();
    async_task<int> t3 = disconnectTask();

    wait_all_awaitable< async_task<int> > wa({ &t1, &t2, &t3 });
    co_await wa;
}