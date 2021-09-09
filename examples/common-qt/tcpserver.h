#ifndef _TCPSERVER_H_
#define _TCPSERVER_H_

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>

#include "connectioninfo.h"

class TcpServer : public QObject
{
    Q_OBJECT

signals:
    void readyReadTcpSig(QTcpSocket* socket, QByteArray& data);
    void newTCPConnectionSig();
    void disconnectedClientSig();

public slots:
    void newTCPConnection();
    void readyReadTcp();
    void stateChanged(QAbstractSocket::SocketState socketState);
    void disconnectedClient();
    void acceptError(QAbstractSocket::SocketError socketError);

public:
    TcpServer(const QString& name = "");
    void configure();
    //void sendMessage(int idx, QByteArray& message);
    void sendMessage(QTcpSocket* sock, QByteArray& message);
    void startListening(quint16 port);
    void closeConnection();

private: // functions
    void enableKeepAlive(QTcpSocket *socket);
    void closeConnection(QTcpSocket *socket);

private:
    QTcpServer              m_TcpServer;
    QList<ConnectionInfo *> m_connectionInfoList;
    QString                 m_name;
};

#endif
