/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPSERVER01_H
#define TCPSERVER01_H

#include <QObject>
#include <QTimer>

#include "tcpconfig.h"
#include "protocolmessage.h"
#include "connectioninfo.h"
#include "tcpclient.h"
#include "tcpserver.h"

class TcpServer01 : public QObject
{
    Q_OBJECT

public:
    explicit TcpServer01(QObject *parent = nullptr, MessageCheck check = NO_CHECK);

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

private:
    QString                 m_serverName;
    QString                 m_serverHost;
    quint16                 m_serverPort;

    TcpServer               m_tcpServer;
    ProtocolMessage         m_message;

    int                     m_errorCounter;
};

#endif // TCPSERVER01_H
