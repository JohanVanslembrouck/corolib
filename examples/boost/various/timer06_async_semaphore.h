/**
 * @file timer06_async_semaphore.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER06_ASYNC_SEMAPHORE_H
#define TIMER06_ASYNC_SEMAPHORE_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/async_semaphore.h>
#include <corolib/commservice.h>

using namespace corolib;

class Timer06 : public CommService
{
public:
    Timer06(boost::asio::io_context& ioContext);
    async_task<void> mainTask();

protected:
    async_operation<void> start_timer(steady_timer& timer, int ms);
    void start_timer_impl(const int idx, steady_timer& tmr, int ms);

    async_task<void> subTask1(int instance, int timeout);
    async_task<void> subTask2(int instance, int timeout);

private:
    boost::asio::io_context& m_ioContext;
    async_semaphore m_semaphore;
};

#endif
