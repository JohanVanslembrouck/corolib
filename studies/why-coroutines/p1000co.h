/**
 * @file p1000co.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1000CO_H_
#define _P1000CO_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

#include "p1000.h"

//extern RemoteObject1 remoteObj1;

using namespace corolib;

class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}
    
    // User API
    /**
     * @brief op1 has same parameter list as op1 in class RemoteObject1
     * but returns async_task<int> instead of int
     * @param in1
     * @param in2
     * @param out2
     * @param out2
     * @return async_task<int>
     */
    async_task<int> op1(int in1, int in2, int& out1, int& out2)
    {
        async_operation<op1_ret_t> op1 = start_op1(in1, in2);
        op1_ret_t res = co_await op1;
        out1 = res.out1;
        out2 = res.out2;
        co_return res.ret;
    }
    
    // Start-up function
    /**
     * @brief auxiliary function for op1, but also accessible to the user.
     * The output parameters and return value are still "packed" in an op1_ret_t object. 
     * @param in1
     * @param in2
     * @return async_operation<op1_ret_t>
     */
    async_operation<op1_ret_t> start_op1(int in1, int in2)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in1, in2);
        return { this, index };
    }

protected:
    // Implementation function
    void start_op1_impl(const int idx, int in1, int in2);

private:
    RemoteObject1 m_remoteObject;
};

#endif
