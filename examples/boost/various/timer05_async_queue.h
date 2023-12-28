/**
 * @file timer05_async_queue.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER05_ASYNC_QUEUE_H
#define TIMER05_ASYNC_QUEUE_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/async_queue.h>
#include <corolib/commservice.h>

using namespace corolib;

const int QUEUESIZE = 32;

const int NR_OPERATIONS = 100;
const int MULTIPLIER = 5;

class Timer05 : public CommService
{
public:
    Timer05(boost::asio::io_context& ioContext);
    async_task<void> mainTasks();

protected:
    async_operation<void> start_timer(steady_timer& timer, int ms);
    void start_timer_impl(const int idx, steady_timer& tmr, int ms);

    async_task<void> mainTask1();
    async_task<void> mainTask2();

    async_task<void> producer(int timeout);
    async_task<void> consumer(int timeout);

private:
    async_queue<int, QUEUESIZE> m_queue;
    boost::asio::io_context& m_ioContext;
};

#endif
