/**
 * @file tcpconfigfile.cpp
 * @brief Contains code to read a file with information to configure a TCP client or server application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QFile>
#include <QTextStream>
#include <QDebug>

#include "tcpconfigfile.h"
#include "tcpconfig.h"

/**
 * @brief TcpConfigFile::readConfigurationFile
 * @param fileName
 * @return
 */
bool TcpConfigFile::readConfigurationFile(QString fileName)
{
    bool fileRead = false;
    QFile inputFile(fileName);

    if (inputFile.open(QIODevice::ReadOnly))
    {
        qInfo() << "Opened configuration file " << fileName;

        QTextStream in(&inputFile);

        while (!in.atEnd())
        {
            QString line = in.readLine();
            qInfo() << line;

            if (line.isEmpty())
                continue;
            if (line.length() >= 1 && line[0] == '#')
                continue;
            if (line.length() >= 2 && (line[0] == '/' && line[1] == '/'))
                continue;

            line.replace(" =", "=");
            line.replace("= ", "=");

            if (line.contains("=", Qt::CaseInsensitive))
            {
                // Get Key and key content
                QStringList splitString = line.split('=');
                QString key = splitString.at(0);
                QString value = splitString.at(1);

                if (QString::compare(key, "serverName", Qt::CaseInsensitive) == 0)
                {
                    configuration.m_serverName = value;
                }

                if (QString::compare(key, "server", Qt::CaseInsensitive) == 0)
                {
                    QStringList ipAddressAndPort = value.split(':');
                    QString ipAddress = ipAddressAndPort[0];
                    quint16 port = ipAddressAndPort[1].toUShort();

                    configuration.m_indexLastValidConfiguration++;

                    configuration.m_server.m_ipAddress = ipAddress;
                    configuration.m_server.m_port = port;

                    int i = configuration.m_indexLastValidConfiguration;
                    configuration.m_servers[i].m_ipAddress = ipAddress;
                    configuration.m_servers[i].m_port = port;
                }

                // Timers
                // ------
                if (QString::compare(key, "startupDelay", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 startupDelay = value.toInt(&ok);
                    if (ok)
                        configuration.m_startupDelay = startupDelay;
                }

                if (QString::compare(key, "delayBeforeReply", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 delayBeforeReply = value.toInt(&ok);
                    if (ok)
                        configuration.m_delayBeforeReply = delayBeforeReply;
                }

                if (QString::compare(key, "waitForConnectionTimeout", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 waitForConnectionTimeout = value.toInt(&ok);
                    if (ok)
                        configuration.m_waitForConnectionTimeout = waitForConnectionTimeout;
                }

                if (QString::compare(key, "reconnectTimeout", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 reconnectTimeout = value.toInt(&ok);
                    if (ok)
                        configuration.m_reconnectTimeout = reconnectTimeout;
                }

                if (QString::compare(key, "reconnectTimeoutAfterDisconnect", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 reconnectTimeoutAfterDisconnect = value.toInt(&ok);
                    if (ok)
                        configuration.m_reconnectTimeoutAfterDisconnect= reconnectTimeoutAfterDisconnect;
                }

                // Display
                // -------
                if (QString::compare(key, "displayInfoMessages", Qt::CaseInsensitive) == 0)
                {
                    if (QString::compare(value, "y", Qt::CaseInsensitive) == 0)
                        configuration.m_displayInfoMessages = true;
                    else
                        configuration.m_displayInfoMessages = false;
                }

                if (QString::compare(key, "numberMessages", Qt::CaseInsensitive) == 0)
                {
                    configuration.m_numberMessages = value.toInt();
                }

                if (QString::compare(key, "numberTransactions", Qt::CaseInsensitive) == 0)
                {
                    configuration.m_numberTransactions = value.toInt();
                }

                if (QString::compare(key, "step", Qt::CaseInsensitive) == 0)
                {
                    configuration.m_step = value.toInt();
                }

                if (QString::compare(key, "useCoroutines", Qt::CaseInsensitive) == 0)
                {
                    if (QString::compare(value, "y", Qt::CaseInsensitive) == 0)
                        configuration.m_useCoroutines = true;
                    else
                        configuration.m_useCoroutines = false;
                }

                if (QString::compare(key, "useAsyncConnect", Qt::CaseInsensitive) == 0)
                {
                    if (QString::compare(value, "y", Qt::CaseInsensitive) == 0)
                        configuration.m_useAsyncConnect = true;
                    else
                        configuration.m_useAsyncConnect = false;
                }

                if (QString::compare(key, "selectImplementation", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 selectImplementation = value.toInt(&ok);
                    if (ok)
                        configuration.m_selectImplementation = selectImplementation;
                }

                if (QString::compare(key, "selectMeasurementLoop", Qt::CaseInsensitive) == 0)
                {
                    bool ok;
                    qint32 selectMeasurementLoop = value.toInt(&ok);
                    if (ok)
                        configuration.m_selectMeasurementLoop = selectMeasurementLoop;
                }
            }
        } // while (!in.atEnd())
        inputFile.close();
        fileRead = true;
    }
    return fileRead;
}
