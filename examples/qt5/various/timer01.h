/**
 * @file timer01.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER01_H
#define TIMER01_H

#include <QObject>
#include <QTimer>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

using namespace corolib;

class Timer01 : public QObject, public CommService
{
    Q_OBJECT

public:
    explicit Timer01(QObject *parent = nullptr);
    async_task<int> mainTask();

protected:
    async_operation<void> start_timer(QTimer& timer, int ms, bool doDisconnect = false);
    void start_timer_impl(const int idx, QTimer& tmr, int ms, bool doDisconnect = false);

    async_task<int> timerTask01();
    async_task<int> timerTask02();
    async_task<int> timerTask03();
    async_task<int> timerTask04();

private:
    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TIMER01_H
