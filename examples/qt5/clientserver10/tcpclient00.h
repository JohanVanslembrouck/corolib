/**
 * @file tcpclient00.h
 * @brief
 * First TCP client application.
 * It uses a TcpClient data member for the communication with the server application.
 * It uses coroutine functionality.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCLIENT00_H
#define TCPCLIENT00_H

#include <QObject>
#include <QTimer>

#include <chrono>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"

class TcpClient00 : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient00(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

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
    QByteArray prepareMessage(int selection);
    void calculateElapsedTime(std::chrono::high_resolution_clock::time_point start);
    void configureTCP();

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
};

#endif // TCPCLIENT01_H
