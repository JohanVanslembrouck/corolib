/**
 * @file tcpconfig.h
 * @brief
 * Defines a struct with configuration information used by TCP client and server applications.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCONFIG_H
#define TCPCONFIG_H

#include <QString>
#include <QtGlobal>

const quint16 SERVERPORT = 22334;
const int MAX_NR_UAVS = 8;

struct IPaddressAndPort
{
    QString     m_ipAddress;
    quint16     m_port;
};

struct TcpConfiguration
{
    qint32          m_numberTransactions = 100;
    qint32          m_numberMessages = 40;
    qint32          m_step = 5;

    bool            m_displayInfoMessages = false;
    bool            m_displayProtocolMessages = false;

    QString         m_serverName = "localhost";
    IPaddressAndPort m_server = { "localhost", 22334 };

    int              m_indexLastValidConfiguration = -1;
    IPaddressAndPort m_servers[32];

    qint32          m_startupDelay = 0;
    qint32          m_delayBeforeReply = 2;

    qint32          m_waitForConnectionTimeout = 1000;
    qint32          m_reconnectTimeout = 5000;
    qint32          m_reconnectTimeoutAfterDisconnect = 500;

    bool            m_useCoroutines = false;
    bool            m_useAsyncConnect = false;
    qint32          m_selectImplementation = 1;
    qint32          m_selectMeasurementLoop = 0;
};

extern TcpConfiguration configuration;

#endif // TcpCONFIG_H
