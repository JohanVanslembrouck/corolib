/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _TCPCLIENT_H_
#define _TCPCLIENT_H_

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QList>
#include <QTimer>

#include "connectioninfo.h"

class TcpClient : public QObject
{
    Q_OBJECT

signals:
    void readyReadTcpSig(QByteArray& data);
    void disconnectedServerSig();

public slots:
    void readyReadTcp();
    void disconnectedServer();
    void stateChanged(QAbstractSocket::SocketState socketState);
    void connectToServerTimed();

public:
    TcpClient(const QString& name = "",
              bool autoConnect = true,
              qint32 waitForConnectionTimeout = 1000,
              qint32 reconnectTimeout = 5000,
              qint32 reconnectTimeoutAfterDisconnect = 500
            );
    void configure();
    void disconnectFromServer();
    bool connectToServer(QString& serverIpAddress, quint16 port);
    void sendMessage(QByteArray& message);

private:    // functions
    void enableKeepAlive(QTcpSocket *socket);
    void closeConnection(QTcpSocket *socket);

private:
    QList<ConnectionInfo *> m_connectionInfoList;

    QTimer      m_timer;

    QString     m_serverIPaddress;
    quint16     m_serverPort;

    bool        m_autoConnect;
    QString     m_name;

    qint32      m_waitForConnectionTimeout;
    qint32      m_reconnectTimeout;
    qint32      m_reconnectTimeoutAfterDisconnect;
};

#endif
