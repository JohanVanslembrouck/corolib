/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCLIENT01_H
#define TCPCLIENT01_H

#include <QObject>
#include <QTimer>

#include <chrono>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"

#include "corolib/async_operation.h"
#include "corolib/commservice.h"
#include "corolib/async_task.h"

using namespace corolib;

class TcpClient01 : public QObject, public CommService
{
    Q_OBJECT
public:
    explicit TcpClient01(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

signals:
    void responseReceivedSig(QByteArray msg);

public slots:
    void start();
    void quit();

    void connectToServer();
    void acceptError(QAbstractSocket::SocketError socketError);
    void stateChanged(QAbstractSocket::SocketState socketState);
    void connected();
    void errorOccurred(QAbstractSocket::SocketError socketError);
    void hostFound();

private slots:
    void sendTCPStart();
    void noResponseReceived();
    void readyReadTcp(QByteArray& data);
    void addErrorMessage(const QString &message);
    void addWarningMessage(const QString &message);
    void addInfoMessage(const QString &message);

private:    // functions
    void connectToServerDelayed();
    void connectToTCPServer(QString& serverIPaddress, quint16 serverPort);
    QByteArray prepareMessage(int selection);
    void calculateElapsedTime();
    void configureTCP();

    // Coroutine related
    async_operation<QByteArray> start_reading();
    void start_read(const int idx);
    async_task<int> measurementLoop0();  // coroutine
    async_task<int> measurementLoop1();  // coroutine
    async_task<int> measurementLoop2();  // coroutine
    async_task<int> measurementLoop3();  // coroutine
    async_task<int> measurementLoop4();  // coroutine

    IPaddressAndPort        m_server;

    int                     m_counter;
    int                     m_errorCounter;
    int                     m_selection;
    int                     m_loop;

    QTimer                  m_timerConnectToServer;
    QTimer                  m_timerStartSending;
    QTimer                  m_timerNoResponse;

    TcpClient               m_tcpClient;
    ProtocolMessage         m_message;

    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;

    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TCPCLIENT01_H
