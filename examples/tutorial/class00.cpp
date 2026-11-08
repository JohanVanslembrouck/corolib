/**
 * @file class00.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck
 */

#include "class00.h"

#include "eventqueue.h"
#include "eventqueuethr.h"

using namespace corolib;

EventQueueFunctionVoidInt eventQueue;
EventQueueThrFunctionVoidInt eventQueueThr;

std::function<void(int)> eventHandler;        // Will be initialized in async_op

UseMode useMode;

int delay = 10;
int delay2 = 20;

ThreadAwaker* awaker = nullptr;

/**
 * @brief async_op
 * See explanation in use_mode.h.
 *
 */
void async_op(std::function<void(int)>&& completionHandler)
{
    switch (useMode)
    {
    case UseMode::USE_NONE:
        print(PRI1, "async_op(): UseMode::USE_NONE\n");
        // Nothing to be done here: eventHandler should be called "manually" by the application
        eventHandler = completionHandler;
        break;
    case UseMode::USE_EVENTQUEUE:
        print(PRI1, "async_op(): UseMode::USE_EVENTQUEUE\n");
        // Not used yet
        eventQueue.push(std::move(completionHandler));
        break;
    case UseMode::USE_THREAD:
    {
        print(PRI1, "async_op(): UseMode::USE_THREAD\n");
        if (awaker)
            awaker->addThread();

        std::thread thread1([completionHandler]() {
            if (delay)
            {
                print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            }

            if (awaker)
                awaker->awaitRelease();

            print(PRI1, "async_op(): thread1: completionHandler(%d);\n", defaultCompletionValue);
            completionHandler(defaultCompletionValue);
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();

        // Normally we should not use delays in an asynchronous application.
        // The following delay is only used to investigate possible problems caused by it,
        // especially when the completionHandler in the thread runs before the following delay expires.
#if 0
        print(PRI1, "async_op(): before std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay2);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay2));
        print(PRI1, "async_op(): after std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay2);
#endif
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "async_op(): UseMode::USE_THREAD_QUEUE\n");
        eventQueueThr.incrementPushCounter();

        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));

            std::function<void(int)> completionHandler1 = completionHandler;
            print(PRI1, "async_op(): thread1: eventQueueThr.push(std::move(completionHandler1));\n");
            eventQueueThr.push(std::move(completionHandler1));
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "async_op(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        completionHandler(defaultCompletionValue);
        break;
    }
}

/**
 * @brief start_operation_impl simulates starting an asynchronous operation;
 *
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [&op](int i)
        {
            if (i >= 0)
            {
                print(PRI1, "completionHandler(): op->set_result(%d)\n", i);
                op.set_result(i);
            }
            else
            {
                print(PRI1, "completionHandler(): op->set_error(%d)\n", i);
                op.set_error(i);
            }
            op.completed();
        });
}
