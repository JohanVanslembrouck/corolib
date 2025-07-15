/**
 * @file p1610.cpp
 * @brief
 * Illustrates how to avoid invoking an asynchronous operation again
 * if the operation is already ongoing or has completed (and its result is available).
 * In case the operation is ongoing, it suffices to co_await its completion.
 * Several co_await statements thus await the completion of a single operation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#include "p1610.h"

async_task<int> Class1490::coroutine1(int i)
{
    print(PRI1, "coroutine1()\n");
    int v1 = 0;
  
    print(PRI1, "coroutine1(): async_operation<int> op1 = m_object.start_operation1();\n");
    async_operation<int> op1 = m_object.start_operation1();
    print(PRI1, "coroutine1(): v1 = co_await op1;\n");
    v1 = co_await op1;

    if (op2_state == OperationState::not_started)
    {
        print(PRI1, "coroutine1(): op2_state == OperationState::not_started\n", m_v2);
        op2_state = OperationState::running;
        m_op2 = m_object.start_operation2_rmc(5);
        print(PRI1, "coroutine1(): m_v2 = co_await op2;\n");
        m_v2 = co_await m_op2;
        print(PRI1, "coroutine1(): m_v2 = %d\n", m_v2);
        op2_state = OperationState::completed;
    }
    else if (op2_state == OperationState::running)
    {
        print(PRI1, "coroutine1(): op2_state == OperationState::running\n", m_v2);
        print(PRI1, "coroutine1(): int v2 = co_await op2;\n");
        int v2 = co_await m_op2;
        print(PRI1, "coroutine1(): v2 = %d\n", v2);
    }
    else
    {
        print(PRI1, "coroutine1(): op2_state == OperationState::completed\n", m_v2);
    }

    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v1 + m_v2 + i);
    co_return v1 + m_v2 + i;
}
