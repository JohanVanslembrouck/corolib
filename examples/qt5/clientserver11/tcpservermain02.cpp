/**
 * @file tcpservermain02.cpp
 * @brief Contains the main function to start a TcpServer02 application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QCoreApplication>
#include <QLoggingCategory>

#include "tcpconfigfile.h"
#include "tcpserver02.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[])
{
    QT_USE_NAMESPACE

    corolib::set_priority(0x01);

    QCoreApplication app(argc, argv);

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
        const QString configFileName = "tcpserver02.cfg";

        if (!configFile.readConfigurationFile(configFileName))
        {
            qInfo() << "Could not open configuration file - using default settings";
        }
    }

    if (!configuration.m_displayInfoMessages)
        QLoggingCategory::setFilterRules(QStringLiteral("*.info=false"));

    TcpServer02 tcpServer02(0, USE_CRC);
    async_task<int> t = tcpServer02.mainTask();

    return app.exec();
}
