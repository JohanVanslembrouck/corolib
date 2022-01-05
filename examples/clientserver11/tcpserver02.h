/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPSERVER02_H
#define TCPSERVER02_H

#include <QObject>
#include <QTimer>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpserver.h"

#include "async_operation.h"
#include "commservice.h"
#include "async_task.h"

using namespace corolib;

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
    void start_accept(const int idx, bool doDisconnect = false);
    async_operation<readInfo> start_reading(bool doDisconnect = false);
    void start_read(const int idx, bool doDisconnect = false);
    async_operation<void> start_timer(QTimer& timer, int ms, bool doDisconnect = false);
    void start_tmr(const int idx, QTimer& tmr, int ms, bool doDisconnect = false);
    async_operation<int> start_disconnecting(bool doDisconnect = false);
    void start_disconnect(const int idx, bool doDisconnect = false);

    async_task<int> mainTask();
    async_task<int> acceptTask();
    async_task<int> readTask();
    async_task<int> timerTask();
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
