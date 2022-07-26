/**
 * @file tcpserver02.h
 * @brief
 * Second TCP server application.
 * It uses a TcpServer data member for communication with client applications.
 * It uses coroutine functionality.
 * TcpServer02 uses 3 coroutine with an infinite loop:
 * - acceptTask accepts connections from a client application
 * - readTask reads requests from a client application
 * - disconnectTask handles disconnecitons from a client application
 * The mainTask coroutine waits for these 3 tasks to co_return (which should never be the case).
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPSERVER02_H
#define TCPSERVER02_H

#include <QObject>
#include <QTimer>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpserver.h"

struct readInfo
{
    QTcpSocket* sock;
    QByteArray  data;
};

class TcpServer02 : public QObject, public CommService
{
    Q_OBJECT

public:
    explicit TcpServer02(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

signals:

public slots:
    void start();
    void quit();

    void acceptError(QAbstractSocket::SocketError socketError);
    void stateChanged(QAbstractSocket::SocketState socketState);
    void connected();
    void errorOccurred(QAbstractSocket::SocketError socketError);
    void hostFound();

private slots:
    void readyReadTcp(QTcpSocket* socket, QByteArray& data);

    void newTCPConnection();
    void disconnectedTCPClient();

    void addErrorMessage(const QString &message);
    void addWarningMessage(const QString &message);
    void addInfoMessage(const QString &message);

private:    // functions
    void configureTCP();

    // Coroutine related
    async_operation<int> start_accepting(bool doDisconnect = false);
    void start_accepting_impl(const int idx, bool doDisconnect);
    async_operation<readInfo> start_reading(bool doDisconnect = false);
    void start_reading_impl(const int idx, bool doDisconnect);
    async_operation<void> start_timer(QTimer& timer, int ms, bool doDisconnect = false);
    void start_timer_impl(const int idx, QTimer& tmr, int ms, bool doDisconnect);
    async_operation<int> start_disconnecting(bool doDisconnect = false);
    void start_disconnecting_impl(const int idx, bool doDisconnect);

    // The following are all coroutines
    async_task<int> mainTask();
    async_task<int> acceptTask();
    async_task<int> readTask();
    async_task<int> disconnectTask();

private:
    QString                 m_serverHost;
    quint16                 m_serverPort;

    TcpServer               m_tcpServer;
    ProtocolMessage         m_message;

    int                     m_errorCounter;

    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TCPSERVER02_H
