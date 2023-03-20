/**
 * @file main.cpp
 * @brief
 * Implementation of a TCP client application
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include "mainwindow.h"

#include <iostream>

#include <QLoggingCategory>

#include "tcpconfigfile.h"

using namespace std;

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    TcpConfigFile configFile;

    if (argc > 1)
    {
        // Passing a configuration file via the command line could be useful for testing.
        QString configFileName = argv[1];
        if (!configFile.readConfigurationFile(configFileName))
        {
            qInfo() << "Could not open configuration file passed from the command line - using default settings";
        }
    }
    else
    {
        const QString configFileName = "tcpclient01.cfg";

        if (!configFile.readConfigurationFile(configFileName))
        {
            qInfo() << "Could not open configuration file - using default settings";
        }
    }

    if (!configuration.m_displayInfoMessages)
        QLoggingCategory::setFilterRules(QStringLiteral("*.info=false"));

    //TcpClient01 tcpClient01(0, USE_CRC);
    //tcpClient01.start();

    MainWindow w;
    w.show();
    return a.exec();
}
