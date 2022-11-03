/**
 * @file p1200co.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1200CO_H_
#define _P1200CO_H_

#include "p1200.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

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
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in1, in2);
        return { this, index };
    }

    async_operation<op2_ret_t> start_op2(int in1, int in2)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op2(): index = %d\n", this, index);
        start_op2_impl(index, in1, in2);
        return { this, index };
    }

    async_operation<op1_ret_t> start_op3(int in1)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op3(): index = %d\n", this, index);
        start_op3_impl(index, in1);
        return { this, index };
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