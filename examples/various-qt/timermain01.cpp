/**
 * @file timermain01.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <QCoreApplication>

#include "timer01.h"

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

    Timer01 timer01(0);
    timer01.start();

    return app.exec();
}
