/**
 * @file connectioninfo.h
 * @brief Contains information of a TCP connection.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef CONNECTIONINFO_H
#define CONNECTIONINFO_H

#include <QTcpSocket>

struct ConnectionInfo
{
    QTcpSocket*             m_socket;
    QMetaObject::Connection m_connection_ReadyRead;
    QMetaObject::Connection m_connection_disconnected;
    QMetaObject::Connection m_connection_stateChanged;
    bool                    m_closed;
};

#endif // CONNECTIONINFO_H
