/**
 * @file tcpclientco1.h
 * @brief TCP client class.
 * Uses coroutines. Variant of tcpclienco.h.
 * See README.md file for further explanation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _TCPCLIENTCO1_H_
#define _TCPCLIENTCO1_H_

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QList>
#include <QTimer>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

#include "connectioninfo.h"
#include "protocolmessage.h"

using namespace corolib;

class TcpClientCo1 : public QObject, public CommService
{
    Q_OBJECT

signals:
    void readyReadTcpSig(QByteArray& data);
    void connectedSig();
    void responseReceivedSig(QByteArray data);
    void disconnectedServerSig();

public slots:
    void readyReadTcp();
    void disconnectedServer();
    void stateChanged(QAbstractSocket::SocketState socketState);
    void connectToServerTimed();
    void receiveTimed();

public:
    TcpClientCo1(qint32 selectImplementation = 1,
                 const QString& name = "",
                 bool autoConnect = true,
                 qint32 waitForConnectionTimeout = 1000,
                 qint32 reconnectTimeout = 5000,
                 qint32 reconnectTimeoutAfterDisconnect = 500
            );
    void configure();
    void disconnectFromServer();
    bool connectToServer(QString& serverIpAddress, quint16 port);
    void sendMessage(QByteArray& message);

	// Coroutine related
    async_operation<QByteArray> start_reading();	// no doDisconnect parameter compared with tcpclientco.h
    async_operation<void> start_timer(QTimer& timer, int ms);
    async_operation<void> start_connecting(QString& serverIpAddress, quint16 port);

    void stop_reading(int idx);
    void stop_timer(int idx);

protected:    // functions
    void enableKeepAlive(QTcpSocket *socket);
    void closeConnection(QTcpSocket *socket);

    void start_reading_impl(const int idx);			// no doDisconnect parameter compared with tcpclientco.h
    void start_timer_impl(const int idx, QTimer& tmr, int ms);
    void start_connecting_impl(const int idx, QString& serverIpAddress, quint16 port);

    void readyReadTcpCo(QByteArray& data);
    void readyReadTcpCo1(QByteArray& data);
    void readyReadTcpCo2(QByteArray& data);

private:
    qint32      m_selectImplementation;

    QList<ConnectionInfo *> m_connectionInfoList;

    QTimer      m_timer;
    QTimer      m_receiveTimer;

    QString     m_serverIPaddress;
    quint16     m_serverPort;

    bool        m_autoConnect;
    QString     m_name;

    qint32      m_waitForConnectionTimeout;
    qint32      m_reconnectTimeout;
    qint32      m_reconnectTimeoutAfterDisconnect;

    ProtocolMessage m_message;
    QByteArray      m_data;

    QMetaObject::Connection m_connections[NROPERATIONS];
	
    // The following data members are new compared with tcpclientco.h
	QMetaObject::Connection m_connection_connect;
	int m_index_connect;
	
	QMetaObject::Connection m_connection_read;
	int m_index_read;

	QMetaObject::Connection m_connection_timer;
	int m_index_timer;
};

#endif
