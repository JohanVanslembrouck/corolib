/**
 * @file tcpclientmain03.cpp
 * @brief Contains the main function to start a TcpClient02 application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QCoreApplication>
#include <QLoggingCategory>

#include "tcpconfigfile.h"
#include "tcpclient03.h"

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
        const QString configFileName = "tcpclient03.cfg";

        if (!configFile.readConfigurationFile(configFileName))
        {
            qInfo() << "Could not open configuration file - using default settings";
        }
    }

    if (!configuration.m_displayInfoMessages)
        QLoggingCategory::setFilterRules(QStringLiteral("*.info=false"));

    TcpClient03 tcpClient03(0, USE_CRC);
    async_task<int> t = tcpClient03.mainTask();

    int res = app.exec();
    return t.get_result();
}
