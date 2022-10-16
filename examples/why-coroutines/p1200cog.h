/**
 * @file p1200cog.h
 * @brief coroutine wrapper class for RemoteObject1 with generic completion handler
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1200COG_H_
#define _P1200COG_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

//#include "common.h"
//#include "variables.h"
//#include "eventqueue.h"
//#include "buf+msg.h"

#include "p1200.h"

using namespace corolib;

class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}

    // User API
    // --------
    
    async_task<int> op1(int in1, int in2, int& out1, int& out2)
    {
        async_operation<op1_ret_t> op1 = start_op1(in1, in2);
        op1_ret_t res = co_await op1;
        out1 = res.out1;
        out2 = res.out2;
        co_return res.ret;
    }
    
    async_task<int> op2(int in1, int in2, int& out1)
    {
        async_operation<op2_ret_t> op2 = start_op2(in1, in2);
        op2_ret_t res = co_await op2;
        out1 = res.out1;
        co_return res.ret;
    }
    
    async_task<int> op3(int in1, int& out1, int& out2)
    {
        async_operation<op1_ret_t> op3 = start_op3(in1);
        op1_ret_t res = co_await op3;
        out1 = res.out1;
        out2 = res.out2;
        co_return res.ret;
    }
    
    // Start-up functions
    // ------------------
    
    async_operation<op1_ret_t> start_op1(int in1, int in2)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in1, in2);
        return { this, index };
    }

    async_operation<op2_ret_t> start_op2(int in1, int in2)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op2(): index = %d\n", this, index);
        start_op2_impl(index, in1, in2);
        return { this, index };
    }

    async_operation<op1_ret_t> start_op3(int in1)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op3(): index = %d\n", this, index);
        start_op3_impl(index, in1);
        return { this, index };
    }

    template<class TYPE>
    void genericCompletionHandler(int idx, TYPE in)
    {
        print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<TYPE>* om_async_operation_t =
            dynamic_cast<async_operation<TYPE>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d): om_async_operation_t->set_result()\n", this, idx);
            om_async_operation_t->set_result(in);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    }

    // Lower level functions (also defined in p1200.h, but with a different lambda type)
    // ---------------------
    
    void sendc_op1(int in1, int in2, lambda_op1_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op1(%d, %d, l)\n", in1, in2);
        registerCB(lambda);
    }

    void sendc_op2(int in1, int in2, lambda_op2_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op2(%d, %d, l)\n", in1, in2);
        registerCB(lambda);
    }

    void sendc_op3(int in1, lambda_op1_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op3(%d, l)\n", in1);
        registerCB(lambda);
    }

protected:
    // Implementation functions
    // ------------------------
    
    void start_op1_impl(const int idx, int in1, int in2);
    void start_op2_impl(const int idx, int in1, int in2);
    void start_op3_impl(const int idx, int in1);

private:
    RemoteObject1 m_remoteObject;
};

#endif
