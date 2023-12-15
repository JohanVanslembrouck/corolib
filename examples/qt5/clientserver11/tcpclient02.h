/**
 * @file tcpclient02.h
 * @brief
 * Second TCP client application.
 * It uses two TcpClientCo data members for the communication with two server applications.
 * It uses coroutine functionality.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCLIENT02_H
#define TCPCLIENT02_H

#include <QObject>
#include <QTimer>

#include <chrono>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"
#include "tcpclientco.h"

class TcpClient02 : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient02(QObject *parent = nullptr, MessageCheck check = NO_CHECK);
    async_task<int> mainTask();

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
    void connectToTCPServer(QString& serverIPaddress, quint16 serverPort);
    QByteArray prepareMessage(int selection, int repetition = 1);
    void calculateElapsedTime(std::chrono::high_resolution_clock::time_point start,
                              int messageLength);
    void configureTCP();

    // The following are all coroutines
    async_task<int> connectToServerAsync();

    async_task<int> measurementLoop0(int selection);
    async_task<int> measurementLoop1(int selection);
    async_task<int> measurementLoop2(int selection);

    async_task<int> measurementLoop10(int selection);
    async_task<int> measurementLoop11(int selection);
    async_task<int> measurementLoop12(int selection);
    async_task<int> measurementLoop13(int selection);
    async_task<int> measurementLoop14(int selection);
    async_task<int> measurementLoop15(int selection);
    async_task<int> measurementLoop16(int selection);

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

    async_task<int> measurementLoop40(TcpClientCo& tcpClient);
    async_task<int> measurementLoop41();
    async_task<int> measurementLoop42();
    async_task<int> measurementLoop43();
    async_task<int> measurementLoop44();

    async_task<int> measurementLoop50(TcpClientCo& tcpClient, int nrRepetitions = 1);
    async_task<int> measurementLoop51();
    async_task<int> measurementLoop52();
    async_task<int> measurementLoop53();
    async_task<int> measurementLoop54();
    async_task<int> measurementLoop55();
    async_task<int> measurementLoop56();
    async_task<int> measurementLoop57();

    async_task<int> measurementLoop60(TcpClientCo& tcpClient, int nrRepetitions = 1);
    async_task<int> measurementLoop61();
    async_task<int> measurementLoop62();
    async_task<int> measurementLoop63();
    async_task<int> measurementLoop64();
    async_task<int> measurementLoop65();
    async_task<int> measurementLoop66();
    async_task<int> measurementLoop67();

    async_task<int> measurementLoop70(TcpClientCo& tcpClient, int nrRepetitions = 1, int timeout = 50);
    async_task<int> measurementLoop71();
    async_task<int> measurementLoop72();
    async_task<int> measurementLoop73();
    async_task<int> measurementLoop74();
    async_task<int> measurementLoop75();
    async_task<int> measurementLoop76();
    async_task<int> measurementLoop77();

private:
    IPaddressAndPort        m_servers[2];

    int                     m_counter;
    int                     m_errorCounter;

    QTimer                  m_timerNoResponse;

    TcpClientCo             m_tcpClient1;
    TcpClientCo             m_tcpClient2;
    ProtocolMessage         m_message;

    std::chrono::high_resolution_clock::time_point m_start;
};

#endif // TCPCLIENT02_H
