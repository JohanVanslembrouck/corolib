/**
 * @file tcpconfigfile.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TCPCONFIGFILE_H
#define TCPCONFIGFILE_H

#include <QString>

class TcpConfigFile
{
public:
    bool readConfigurationFile(QString filename);
};

#endif // TCPCONFIGFILE_H
