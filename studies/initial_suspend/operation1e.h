/**
 * @file operation1e.h
 * @brief
 * Class operation1e starts an operation in an eager way, in this implementation in the constructor of operation1e.
 * (The e stands for eager.)
 * This class uses a mini_awaiter_ts data member to protect data in mini_awaiter_ts against concurrent access.
 * The reason is that start_operation1() may access data in the ma object from the completions thread
 * at the moment the launching thread calls co_await (which translates to calls of await_ready(), await_suspend() and await_resume())
 * to check if the operation has already completed.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _OPERATION1E_H_
#define _OPERATION1E_H_

#include <coroutine>

#include "print.h"
#include "tracker1.h"
#include "mini_awaiter_ts.h"

class operation1e
{
public:
    operation1e(bool delayafterstart = false) :
        m_delayafterstart(delayafterstart)
    {
        print(PRI1, "operation1e::operation1e(...)\n");
        start_operation1();         // started from constructor
    }

    // The following 3 functions can be placed in a base class that is inherited by
    // operation1e, operation2e and operation3e.
    bool await_ready() {
        print(PRI1, "operation1e::await_ready(): returns %d\n", ma.await_ready());
        return ma.await_ready();
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI1, "operation1e::await_suspend()\n");
        ma.await_suspend(awaiting);
    }

    int await_resume() {
        print(PRI1, "operation1e::await_resume()\n");
        return ma.await_resume();
    }

protected:
    void start_operation1() {
        print(PRI1, "operation1e::start_operation1()\n");

        // Launch the operation (not shown)

        // The completion of the operation runs on another thread, in this case after 1000 ms
        std::thread thread1([this]() {
            print(PRI1, "operation1e::start_operation1(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "operation1e::start_operation1(): thread1: this->set_result_and_resume(10);\n");
            this->ma.set_result_and_resume(10);
            print(PRI1, "operation1e::start_operation1(): thread1: return;\n");
            });
        thread1.detach();

        if (m_delayafterstart) {
            print(PRI1, "operation1e::start_operation1(): std::this_thread::sleep_for(std::chrono::milliseconds(1500));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    }
    
private:
    bool m_delayafterstart = false;
    mini_awaiter_ts ma;
};

operation1e start_operation1(bool delayafterstart = false)
{
    return operation1e{ delayafterstart };
}

#endif
