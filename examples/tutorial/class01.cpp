/**
 *  Filename: class01.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#include "class01.h"

#include <corolib/print.h>

async_operation<int> Class01::start_operation()
{
    index = (index + 1) & (NROPERATIONS - 1);
    print(PRI1, "%p: Class01::start_operation(): index = %d\n", this, index);
    async_operation<int> ret{ this, index };
    start_op(index);
    return ret;
}

void Class01::start_op(const int idx)
{
    print(PRI1, "%p: Class01::start_op(%d)\n", this, idx);

    operation = [this, idx](int i)
    {
        print(PRI1, "%p: Class01::start_op(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<int>* om_async_operation_t =
            dynamic_cast<async_operation<int>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: Class01::start_op(%d): om_async_operation_t->set_result(%d)\n", this, idx, i);
            om_async_operation_t->set_result(i);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: Class01::start_op(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    if (m_useMode == USE_EVENTQUEUE)
    {
        eventQueue.push(operation);
    }
    if (m_useMode == USE_THREAD)
    {
        std::thread thread1([this]() {
            print(PRI1, "Class01::start_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "Class01::start_op(): thread1: this->operation(10);\n");
            this->operation(10);
            print(PRI1, "Class01::start_op(): thread1: return;\n");
            });
        thread1.detach();
    }
}

EventQueue eventQueue;
