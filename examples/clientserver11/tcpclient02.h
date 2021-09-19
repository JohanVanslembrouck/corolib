/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCLIENT02_H
#define TCPCLIENT02_H

#include <QObject>
#include <QTimer>

#include <chrono>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"
#include "tcpclientco.h"

#include "corolib/async_operation.h"
#include "corolib/commservice.h"
#include "corolib/async_task.h"

using namespace corolib;

class TcpClient02 : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient02(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

signals:
    void responseReceivedSig();

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
    void calculateElapsedTime(int messageLength);
    void configureTCP();

    // The following are all coroutines
    async_task<int> measurementLoop0();
    async_task<int> measurementLoop1();
    async_task<int> measurementLoop2();

    async_task<int> measurementLoop10();
    async_task<int> measurementLoop11();
    async_task<int> measurementLoop12();
    async_task<int> measurementLoop13();
    async_task<int> measurementLoop14();
    async_task<int> measurementLoop15();
    async_task<int> measurementLoop16();

    async_task<int> measurementLoop20();
    async_task<int> measurementLoop21();
    async_task<int> measurementLoop22();

    async_task<int> measurementLoop30();
    async_task<int> measurementLoop31();
    async_task<int> measurementLoop32();
    async_task<int> measurementLoop33();
    async_task<int> measurementLoop34();
    async_task<int> measurementLoop35();
    async_task<int> measurementLoop36();

private:
    QString                 m_serverName;
    QString                 m_serverHost;
    quint16                 m_serverPort;

    int                     m_counter;
    int                     m_errorCounter;
    int                     m_selection;
    int                     m_loop;

    QTimer                  m_timerConnectToServer;
    QTimer                  m_timerStartSending;
    QTimer                  m_timerNoResponse;

    TcpClientCo             m_tcpClient1;
    TcpClientCo             m_tcpClient2;
    ProtocolMessage         m_message;

    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;
};

#endif // TCPCLIENT02_H
