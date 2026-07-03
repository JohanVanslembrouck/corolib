/**
 * @file operation3e.h
 * @brief
 * Class operation3e starts an operation in an eager way, in this implementation in the constructor of operation3e.
 * (The e stands for eager.)
 * This class uses a mini_awaiter_ts data member to protect data in mini_awaiter_ts against concurrent access.
 * The reason is that start_operation1() may access data in the ma object from the completions thread
 * at the moment the launching thread calls co_await (which translates to calls of await_ready(), await_suspend() and await_resume())
 * to check if the operation has already completed.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _OPERATION3E_H_
#define _OPERATION3E_H_

#include <coroutine>

#include "print.h"
#include "tracker1.h"
#include "mini_awaiter_ts.h"
#include "threadawaker.h"

class operation3e
{
public:
    operation3e(ThreadAwaker* awaker, bool delayafterstart = false)
        : m_awaker(awaker)
        ,  m_delayafterstart(delayafterstart)
    {
        print(PRI1, "operation3e::operation3e(...)\n");
        start_operation3();         // started from constructor
    }

    // The following 3 functions can be placed in a base class that is inherited by
    // operation1e, operation2e and operation3e.
    bool await_ready() {
        print(PRI1, "operation3e::await_ready(): returns %d\n", ma.await_ready());
        return ma.await_ready();
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI1, "operation3e::await_suspend()\n");
        ma.await_suspend(awaiting);
    }

    int await_resume() {
        print(PRI1, "operation3e::await_resume()\n");
        return ma.await_resume();
    }

protected:
    void start_operation3() {
        print(PRI1, "operation3e::start_operation3()\n");

        // Launch the operation (not shown)

        if (m_awaker)
            m_awaker->addThread();

        // The completion of the operation runs on another thread, in this case after 1000 ms
        std::thread thread1([this]() {
            print(PRI1, "operation3e::start_operation3(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            if (m_awaker)
                m_awaker->awaitRelease();

            print(PRI1, "operation3e::start_operation3(): thread1: this->set_result_and_resume(10);\n");
            this->ma.set_result_and_resume(10);
            print(PRI1, "operation3e::start_operation3(): thread1: return;\n");
            });
        thread1.detach();

        if (m_delayafterstart) {
            print(PRI1, "operation3e()::start_operation3(): std::this_thread::sleep_for(std::chrono::milliseconds(1500));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    }
    
private:
    ThreadAwaker* m_awaker;
    bool m_delayafterstart = false;
    mini_awaiter_ts ma;
};

operation3e start_operation3(ThreadAwaker* awaker, bool delayafterstart = false)
{
    return operation3e{ awaker, delayafterstart };
}

#endif
