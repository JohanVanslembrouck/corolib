/**
 * @file timer08.cpp
 * @brief 
 * See timer08.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "timer08.h"

#include <corolib/when_all.h>

template<int NR_TASKS, typename TYPE>
class TaskHolder
{
public:
    enum class usage_status
    {
        UNUSED = 0,
        RESERVED,
        USED
    };

    void init()
    {
        m_index = -1;
        for (int i = 0; i < NR_TASKS; ++i)
        {
            usages[i] = usage_status::UNUSED;
        }
    }

    TaskHolder()
    {
        init();
    }

    int get_free_index()
    {
        for (int i = 0; i < NR_TASKS; i++)
        {
            m_index = (m_index + 1) & (NR_TASKS - 1);
           
            if (usages[m_index] == usage_status::UNUSED)
            {
                // Found free entry
                print(PRI2, "%p: TaskHolder::get_free_index() returns %d\n", this, m_index);
                usages[m_index] = usage_status::RESERVED;
                return m_index;
            }
        }
        // No more free entries
        return -1;
    }

    void assign(int index, async_task<TYPE>&& task)
    {
        usages[index] = usage_status::USED;
        tasks[index] = std::move(task);
    }
 
    void inspect()
    {
        for (int i = 0; i < NR_TASKS; i++)
        {
            print(PRI1, "tasks[%d].is_ready() = %d\n", i, tasks[i].is_ready());
        }
    }

public:
    usage_status usages[NR_TASKS];
    async_task<TYPE> tasks[NR_TASKS];
    int m_index;
};

/**
 * @brief Timer08::Timer08
 * @param ioContext
 */
Timer08::Timer08(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_running(true)
{
    print(PRI1, "Timer08::Timer08(...)\n");
}

/**
 * @brief Timer08::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer08::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI1, "%p: Timer08::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer08::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer08::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI1, "%p: Timer08::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer08::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);

            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI1, "%p: Timer08::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer08::start_timer
 * @param async_op
 * @param tmr
 * @param ms
 */
void Timer08::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
    async_operation_base* p_async_op = &async_op;
    
    print(PRI1, "%p: Timer08::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer08::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);
            
            if (!error)
            {
                completionHandler_v(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer08::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer08::clientTask
 * This coroutine starts a timer that simulates the time it takes to serve a client.
 * @param i: identifies a client
 * @return
 */
async_task<int> Timer08::clientTask(int i)
{
    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1 = start_timer(timer1, 1000);

    print(PRI1, "--- Timer08::clientTask(%d): before co_await op_tmr\n", i);
    co_await op_timer1;
    print(PRI1, "--- Timer08::clientTask(%d): after co_await op_tmr\n", i);
    co_return 1;
}

/**
 * @brief Timer08::mainTask
 * This coruutine starts a local timer timer1 and co_awaits its expiration.
 * When the timer expires, mainTask looks for a free entry in taskHolder.
 ° If one is found, mainTask starts coroutine clientTask.
 * If there aren't any free entries anymore, we wait for all tasks to finish.
 * Note: In a real server application, it suffices to wait for one task to finish.
 * 
 * @return
 */
async_task<int> Timer08::mainTask()
{
    print(PRI1, "%p: Timer08::mainTask\n", this);

    const static int NR_TASKS = 16;
    TaskHolder<NR_TASKS, int> taskHolder;

    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);
 
    for (int j = 0; j < 5 * NR_TASKS; ++j)
    {
        start_timer(op_timer1, timer1, 100);
        print(PRI1, "%p: Timer08::mainTask: before co_await op_timer1 100\n", this);
        co_await op_timer1;
        print(PRI1, "%p: Timer08::mainTask: after co_await op_timer1 100\n", this);

        int i = taskHolder.get_free_index();
        if (i == -1)
        {
            // No free entry anymore. Wait for all running tasks to finish.
            taskHolder.inspect();

            print(PRI1, "%p: Timer08::mainTask: when_allT<async_task<int>> wa(tasks, NR_TASKS)\n", this);
            when_allT<async_task<int>> wa(taskHolder.tasks, NR_TASKS);

            print(PRI1, "%p: Timer08::mainTask: before co_await wa;\n", this);
            co_await wa;
            print(PRI1, "%p: Timer08::mainTask: after co_await wa;\n", this);

            taskHolder.inspect();

            taskHolder.init();

            // Look again for a free entry. All should be free at this point.
            i = taskHolder.get_free_index();
            if (i == -1)
            {
                print(PRI1, "%p: Timer08::mainTask: could not find free index!!!\n", this);
            }
        }

        // Start a task and mark its entry as in use
        taskHolder.tasks[i] = std::move(clientTask(j));
        taskHolder.usages[i] = TaskHolder<NR_TASKS, int>::usage_status::USED;
    }

    print(PRI1, "%p: Timer08::mainTask: Wait for all tasks to finish before leaving mainTask\n", this);
    taskHolder.inspect();

    print(PRI1, "%p: Timer08::mainTask: when_allT<async_task<int>> wa(tasks, NR_TASKS)\n", this);
    when_allT<async_task<int>> wa(taskHolder.tasks, NR_TASKS);

    print(PRI1, "%p: Timer08::mainTask: before co_await wa;\n", this);
    co_await wa;
    print(PRI1, "%p: Timer08::mainTask: after co_await wa;\n", this);

    taskHolder.inspect();

    print(PRI1, "--- mainTask: co_return 1;\n");
    co_return 1;
}
