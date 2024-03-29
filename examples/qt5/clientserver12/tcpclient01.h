/**
 * @file tcpclient01.h
 * @brief
 * TCP client application.
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

    void startMeasurementLoop0();
    void startMeasurementLoop1();
    void startMeasurementLoop2();
    void startMeasurementLoop3();
    void startMeasurementLoop4();

    void stopMeasurementLoop0();
    void stopMeasurementLoop1();
    void stopMeasurementLoop2();
    void stopMeasurementLoop3();
    void stopMeasurementLoop4();

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
    void noResponseReceived();
    void readyReadTcp(QByteArray& data);
    void addErrorMessage(const QString &message);
    void addWarningMessage(const QString &message);
    void addInfoMessage(const QString &message);

private:    // functions
    void connectToServerDelayed();
    void connectToTCPServer(QString& serverIPaddress, quint16 serverPort);
    QByteArray prepareMessage(int selection);
    void calculateElapsedTime(std::chrono::high_resolution_clock::time_point start);
    void configureTCP();

    // Coroutine related
    async_operation<QByteArray> start_reading();
    void start_reading_impl(const int idx);
	
    // The following are all coroutines
    async_task<int> measurementLoop0();
    async_task<int> measurementLoop1();
    async_task<int> measurementLoop2();
    async_task<int> measurementLoop3();
    async_task<int> measurementLoop4();

    bool m_measurementLoop0running;
    bool m_measurementLoop1running;
    bool m_measurementLoop2running;
    bool m_measurementLoop3running;
    bool m_measurementLoop4running;

    IPaddressAndPort        m_server;

    int                     m_counter;
    int                     m_errorCounter;

    QTimer                  m_timerConnectToServer;
    QTimer                  m_timerNoResponse;

    TcpClient               m_tcpClient;
    ProtocolMessage         m_message;

    std::chrono::high_resolution_clock::time_point m_start;

    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TCPCLIENT01_H
