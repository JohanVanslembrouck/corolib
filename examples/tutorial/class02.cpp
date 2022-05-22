/**
 *  Filename: class02.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#include "class02.h"

#include <corolib/print.h>

async_operation<int> Class02::start_operation1()
{
    int index = get_free_index();
    print(PRI1, "%p: Class02::start_operation1(): index = %d\n", this, index);
    async_operation<int> ret{ this, index };
    start_op1(index);
    return ret;
}

void Class02::start_op1(const int idx)
{
    print(PRI1, "%p: Class02::start_op1(%d)\n", this, idx);

    operation[idx] = [this, idx](int i)
    {
        print(PRI1, "%p: Class02::start_op1(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<int>* om_async_operation_t =
            dynamic_cast<async_operation<int>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: Class02::start_op1(%d): om_async_operation_t->set_result(%d)\n", this, idx, i);
            om_async_operation_t->set_result(i);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: Class02::start_op1(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    if (m_useMode == USE_EVENTQUEUE)
    {
        eventQueue.push(operation[idx]);
    }
    else if (m_useMode == USE_THREAD)
    {
        std::thread thread1([this, idx]() {
            print(PRI1, "Class02::start_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "Class02::start_op(): thread1: this->operation(10);\n");
            this->operation[idx](10);
            print(PRI1, "Class02::start_op(): thread1: return;\n");
            });
        thread1.detach();
    }
    else if (m_useMode == USE_IMMEDIATE_COMPLETION)
    {
        operation[idx](10);
    }
}

async_operation<int> Class02::start_operation2(int bias)
{
    int index = get_free_index();
    print(PRI1, "%p: Class02::start_operation2(): index = %d\n", this, index);
    async_operation<int> ret{ this, index };
    start_op2(index, bias);
    return ret;
}

void Class02::start_op2(const int idx, int bias)
{
    print(PRI1, "%p: Class02::start_op2(%d, %d)\n", this, idx, bias);

    operation[idx] = [this, idx, bias](int i)
    {
        print(PRI1, "%p: Class02::start_op2(%d, %d)\n", this, idx, bias);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<int>* om_async_operation_t =
            dynamic_cast<async_operation<int>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: Class02::start_op2(%d, %d): om_async_operation_t->set_result(%d)\n", this, idx, bias, bias + i);
            om_async_operation_t->set_result(bias + i);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: Class02::start_op2(%d, %d): Warning: om_async_operation_t == nullptr\n", this, idx, bias);
        }
    };

    if (m_useMode == USE_EVENTQUEUE)
    {
        eventQueue.push(operation[idx]);
    }
    else if (m_useMode == USE_THREAD)
    {
        std::thread thread1([this, idx, bias]() {
            print(PRI1, "Class02::start_op2(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "Class02::start_op2(): thread1: this->operation(10);\n");
            this->operation[idx](10);
            print(PRI1, "Class02::start_op2(): thread1: return;\n");
            });
        thread1.detach();
    }
    else if (m_useMode == USE_IMMEDIATE_COMPLETION)
    {
        operation[idx](10);
    }
}

EventQueue eventQueue;
