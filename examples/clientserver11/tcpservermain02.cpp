/**
 * @file
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
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
    tcpServer02.start();

    return app.exec();
}
