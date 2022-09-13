/**
 * @file p1010-async-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

class RemoteObj1 : public CommService
{
public:
    async_operation<op1_ret_t> m_op1;
    lambda1 m_lambda;
    
    void sendc_op1(int in11, int in12, lambda1 lambda) {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in11, in12);
        m_op1 = start_op1(int in11, int in12);
    }
    
    // Start-up function
    async_operation<op1_ret_t> start_op1(int in11, int in12) {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op1_impl([this]() { this->op_cb() } , index, in11, in12);
        return ret;
    }

    void op_cb(void) {
        printf("RemoteObj1::op_cb(%d, %d, %d)\n", out11, out12, ret1);
        op1_ret_t res = m_op1.get_result();
        
        m_lambda(1, 2, 3);
        printf("RemoteObj1::op_cb(): part 2\n");
    }
    
protected:
    // Implementation function
    void start_op1_impl(const int idx, int in11, int in12);
};

void RemoteObj1::start_op1_impl(lambda_void_t cb, const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

    operation1 = [this, idx](int out11, int out12, int ret1)
    {
        print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<op1_ret_t>* om_async_operation_t =
            dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): om_async_operation_t->set_result()\n", this, idx);
            op1_ret_t op1_ret = { out11, out12, ret1 };
            om_async_operation_t->set_result(op1_ret);
            om_async_operation_t->completed();
            eventQueue.push(cb);
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    eventQueue.push([this]() { this->operation1(1, 2, 3); });
}

// -------------------------------------------------

struct Class01 {
    void function1() {
        printf("Class01::function1(): part 1\n");
        remoteObj1.sendc_op1(in11, in12, 
            [this](int out1, int out2, int ret1) { 
                this->function1_cb(out1, out2, ret1); 
            });
    }

    void function1_cb(int out11, int out12, int ret1) {
        printf("Class01::function1_cb(%d, %d, %d)\n", out11, out12, ret1);
        printf("Class01::function1_cb(): part 2\n");
    }
};

Class01 class01;

// -------------------------------------------------

int main() {
    printf("main();\n");
    connect(event1, []() { class01.function1(); });
    //connect(event2, []() { class01.function1(); });
    eventQueue.run();
    return 0;
}
