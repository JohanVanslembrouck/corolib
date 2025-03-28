/**
 * @file operation1l.h
 * @brief
 * class operation1l starts an operation in a lazy way, in this class at the moment we co_await the operation1l object
 * This class does not have da
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _OPERATION1L_H_
#define _OPERATION1L_H_

#include <coroutine>

#include "print.h"
#include "tracker1.h"
#include "mini_awaiter.h"

class operation1l
{
public:
    operation1l(bool delayafterstart = false) :
        m_delayafterstart(delayafterstart)
    {
    }

    bool await_ready() {
        print(PRI1, "operation1l::await_ready(): returns %d\n", ma.await_ready());
        return ma.await_ready();
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI1, "operation1l::await_suspend()\n");
        ma.await_suspend(awaiting);
        start_operation1();         // called from await_suspend
    }

    int await_resume() {
        print(PRI1, "operation1l::await_resume()\n");
        return ma.await_resume();
    }

protected:
    void start_operation1() {
        // Launch the operation (not shown)

        // The completion of the operation runs on another thread, in this case after 1000 ms
        std::thread thread1([this]() {
            print(PRI1, "operation1l::start_operation1(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "operation1l::start_operation1(): thread1: this->set_result_and_resume(10);\n");
            this->ma.set_result_and_resume(10);
            print(PRI1, "operation1el:start_operation1(): thread1: return;\n");
            });
        thread1.detach();

        if (m_delayafterstart) {
            print(PRI1, "operation1l()::start_operation1(): std::this_thread::sleep_for(std::chrono::milliseconds(1500));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    }

private:
    bool m_delayafterstart = false;
    mini_awaiter ma;
};

#endif
