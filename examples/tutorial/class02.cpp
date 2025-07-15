/**
 * @file class02.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include "class02.h"

#include <corolib/print.h>

Class02::Class02(UseMode useMode,
    EventQueueFunctionVoidInt* eventQueue,
    EventQueueThrFunctionVoidInt* eventQueueThr,
    std::mutex* mtx,
    ThreadAwaker* awaker,
    int delay)
    : m_useMode(useMode)
    , m_eventQueue(eventQueue)
    , m_eventQueueThr(eventQueueThr)
    , m_mutex(mtx)
    , m_awaker(awaker)
    , m_delay(delay)
    , m_queueSize(0)
{
    for (int i = 0; i < NROPERATIONS; ++i)
    {
        m_eventHandler[i] = 
            [this, i](int) {
                print(PRI1, "%p: Class02::invalid m_eventHandler entry: index = %d\n", this, i);
            };
    }
}

async_operation<int> Class02::start_operation1()
{
    int index = get_free_index();
    print(PRI1, "%p: Class02::start_operation1(): index = %d\n", this, index);
    async_operation<int> ret{ this, index };
    start_operation1_impl(index);
    return ret;
}

/**
 * @brief Class02::async_op1
 * See explanation in use_mode.h.
 *
 * @param idx
 */
void Class02::async_op1(const int idx, std::function<void(int)>&& completionHandler)
{
    switch (m_useMode)
    {
    case UseMode::USE_NONE:
        // Nothing to be done: m_eventHandler[idx] should be called "manually" by the application
        m_eventHandler[idx] = completionHandler;
        break;
    case UseMode::USE_EVENTQUEUE:
        if (m_eventQueue)
            m_eventQueue->push(std::move(completionHandler));
        break;
    case UseMode::USE_THREAD:
    {
        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, idx, completionHandler]() {
            print(PRI1, "Class02::async_op1(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "Class02(): async_op1(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            if (m_mutex) {
                std::lock_guard<std::mutex> guard(*m_mutex);
                print(PRI1, "Class02::async_op1(): thread1: completionHandler(10);\n", idx);
                completionHandler(10);
            }
            else {
                print(PRI1, "Class02::async_op1(): thread1: completionHandler(10);\n", idx);
                completionHandler(10);
            }

            print(PRI1, "Class02::async_op1(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        m_queueSize++;

        std::thread thread1([this, completionHandler]() {
            print(PRI1, "Class02::async_op1(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            std::function<void(int)> completionHandler1 = completionHandler;
            print(PRI1, "Class02::async_op1(): thread1: m_eventQueueThr->push(std::move(completionHandler1));\n");
            m_eventQueueThr->push(std::move(completionHandler1));
            print(PRI1, "Class02::async_op1(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        completionHandler(10);
        break;
    }
}

/**
 * @brief Class02::start_operation1_impl simulates starting an asynchronous operation.
 *
 * @param idx
 */
void Class02::start_operation1_impl(const int idx)
{
    print(PRI1, "%p: Class02::start_operation1_impl(%d)\n", this, idx);

    async_op1(idx, 
        [this, idx](int i)
        {
            print(PRI1, "Class02::eventHandler[%p, %d](%d)\n", this, idx, i);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<int>* om_async_operation_t =
                static_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                if (i >= 0)
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_result(%d);\n", this, idx, i, i);
                    om_async_operation_t->set_result(i);
                }
                else
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_error(%d);\n", this, idx, i, i);
                    om_async_operation_t->set_error(i);
                }
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "Class02::eventHandler[%p, %d](%d): Warning: om_async_operation_t == nullptr\n", this, idx, i);
            }
        });
}

async_operation<int> Class02::start_operation2(int bias)
{
    int index = get_free_index();
    print(PRI1, "%p: Class02::start_operation2(): index = %d\n", this, index);
    async_operation<int> ret{ this, index };
    start_operation2_impl(index, bias);
    return ret;
}

async_operation_rmc<int> Class02::start_operation2_rmc(int bias)
{
    int index = get_free_index();
    print(PRI1, "%p: Class02::start_operation2_rmc(): index = %d\n", this, index);
    async_operation_rmc<int> ret{ this, index };
    start_operation2_rmc_impl(index, bias);
    return ret;
}

/**
 * @brief Class02::async_op2
 * See explanation in use_mode.h.
 * 
 */
void Class02::async_op2(int idx, int bias, std::function<void(int)>&& completionHandler)
{
    switch (m_useMode)
    {
    case UseMode::USE_NONE:
        // Nothing to be done: eventHandler[idx] should be called "manually" by the application
        m_eventHandler[idx] = completionHandler;
        break;
    case UseMode::USE_EVENTQUEUE:
        if (m_eventQueue)
            m_eventQueue->push(std::move(completionHandler));
        break;
    case UseMode::USE_THREAD:
    {
        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, idx, bias, completionHandler]() {
            print(PRI1, "Class02::async_op2(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "Class02(): async_op2(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            if (m_mutex) {
                std::lock_guard<std::mutex> guard(*m_mutex);
                print(PRI1, "Class02::async_op2(): thread1: completionHandler(10);\n", idx);
                completionHandler(10);
            }
            else {
                print(PRI1, "Class02::async_op2(): thread1: completionHandler(10);\n", idx);
                completionHandler(10);
            }

            print(PRI1, "Class02::async_op2(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        m_queueSize++;

        std::thread thread1([this, completionHandler]() {
            print(PRI1, "Class02::async_op2(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            std::function<void(int)> completionHandler1 = completionHandler;
            print(PRI1, "Class02::async_op2(): thread1: m_eventQueueThr->push(std::move(completionHandler1));\n");
            m_eventQueueThr->push(std::move(completionHandler1));
            print(PRI1, "Class02::async_op2(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        completionHandler(10);
        break;
    }
}

/**
 * @brief Class02::start_operation2_impl simulates starting an asynchronous operation.
 *
 * @param idx
 * @param bias
 */
void Class02::start_operation2_impl(const int idx, int bias)
{
    print(PRI1, "%p: Class02::start_operation2_impl(%d, %d)\n", this, idx, bias);

    async_op2(idx, bias, 
        [this, idx, bias](int i)
        {
            print(PRI1, "Class02::eventHandler[%p, %d, %d](%d)\n", this, idx, bias, i);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<int>* om_async_operation_t =
                static_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                if (i >= 0)
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_result(%d);\n", this, idx, i, bias + i);
                    om_async_operation_t->set_result(bias + i);
                }
                else
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_error(%d);\n", this, idx, i, i);
                    om_async_operation_t->set_error(i);
                }
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "Class02::eventHandler[%p, %d, %d](%d): Warning: om_async_operation_t == nullptr\n", this, idx, bias, i);
            }
        });
}


void Class02::start_operation2_rmc_impl(const int idx, int bias)
{
    print(PRI1, "%p: Class02::start_operation2_rmc_impl(%d, %d)\n", this, idx, bias);

    async_op2(idx, bias,
        [this, idx, bias](int i)
        {
            print(PRI1, "Class02::eventHandler[%p, %d, %d](%d)\n", this, idx, bias, i);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation_rmc<int>* om_async_operation_t =
                static_cast<async_operation_rmc<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                if (i >= 0)
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_result(%d);\n", this, idx, i, bias + i);
                    om_async_operation_t->set_result(bias + i);
                }
                else
                {
                    print(PRI1, "Class02::eventHandler[%p, %d](%d): om_async_operation_t->set_error(%d);\n", this, idx, i, i);
                    om_async_operation_t->set_error(i);
                }
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "Class02::eventHandler[%p, %d, %d](%d): Warning: om_async_operation_t == nullptr\n", this, idx, bias, i);
            }
        });
}

void Class02::runEventHandler(int i, int val)
{
    std::function<void(int)> eventHandler_ = m_eventHandler[i];
    eventHandler_(val);
    m_eventHandler[i] =
        [this, i](int) {
            print(PRI1, "%p: Class02::invalid m_eventHandler entry: index = %d\n", this, i);
        };
}