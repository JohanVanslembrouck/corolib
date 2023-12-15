/**
 * @file tcpclient01.h
 * @brief
 * First TCP client application.
 * It uses a TcpClient data member for the communication with the server application.
 * It uses coroutine functionality.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCLIENT01_H
#define TCPCLIENT01_H

#include <QObject>
#include <QTimer>

#include <chrono>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"

using namespace corolib;

class TcpClient01 : public QObject, public CommService
{
    Q_OBJECT
public:
    explicit TcpClient01(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

    async_task<int> mainTask();

signals:
    void responseReceivedSig(QByteArray msg);

public slots:
    void quit();
    void acceptError(QAbstractSocket::SocketError socketError);
    void stateChanged(QAbstractSocket::SocketState socketState);
    void errorOccurred(QAbstractSocket::SocketError socketError);
    void hostFound();

private slots:
    void noResponseReceived();
    void readyReadTcp(QByteArray& data);
    void addErrorMessage(const QString &message);
    void addWarningMessage(const QString &message);
    void addInfoMessage(const QString &message);

private:    // functions
    void connectToServer(QString& serverIPaddress, quint16 serverPort);
    QByteArray prepareMessage(int selection);
    void calculateElapsedTime(std::chrono::high_resolution_clock::time_point start);
    void configureTCP();

    // Coroutine related
    async_operation<QByteArray> start_reading();
    async_operation<void> start_timer(QTimer& timer, int ms);
    async_operation<void> start_connecting(QString& serverIpAddress, quint16 port);

    void start_reading_impl(const int idx);
    void start_timer_impl(const int idx, QTimer& tmr, int ms);
    void start_connecting_impl(const int idx, QString& serverIpAddress, quint16 port);

    // The following are all coroutines
    async_task<int> connectToServerAsync();

    async_task<int> measurementLoop0(int selection);
    async_task<int> measurementLoop1(int selection);
    async_task<int> measurementLoop2(int selection);
    async_task<int> measurementLoop3(int selection);
    async_task<int> measurementLoop4(int selection);

    IPaddressAndPort        m_server;

    int                     m_counter;
    int                     m_errorCounter;
    int                     m_loop;

    QTimer                  m_timerConnectToServer;
    QTimer                  m_timerStartSending;
    QTimer                  m_timerNoResponse;

    TcpClient               m_tcpClient;
    ProtocolMessage         m_message;

    std::chrono::high_resolution_clock::time_point m_start;

    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TCPCLIENT01_H
