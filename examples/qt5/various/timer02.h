/**
 * @file timer02.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER02_H
#define TIMER02_H

#include <QObject>
#include <QTimer>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

using namespace corolib;

class Timer02 : public QObject, public CommService
{
    Q_OBJECT

public:
    explicit Timer02(QObject *parent = nullptr);
    void start();

protected:
    async_operation<void> start_timer(QTimer& timer, int ms, bool doDisconnect = false);
    void start_timer_impl(const int idx, QTimer& tmr, int ms, bool doDisconnect = false);
    
    void connect_to_timer(async_operation_base& async_op, QTimer& tmr, QMetaObject::Connection& conn, bool doDisconnect = false);
   
    async_task<int> mainTask();
    async_task<int> timerTask01();
    async_task<int> timerTask02();
    async_task<int> timerTask03();
    async_task<int> timerTask04();

private:
    QMetaObject::Connection m_connections[NROPERATIONS];
};

#endif // TIMER02_H
