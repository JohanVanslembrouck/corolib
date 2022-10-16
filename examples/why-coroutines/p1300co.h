/**
 * @file p1300co.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1300CO_H_
#define _P1300CO_H_

#include "p1300.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}
    
    async_operation<int> start_op1(Msg msg)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, msg);
        return { this, index };
    }

protected:
    void start_op1_impl(const int idx, Msg& msg);

private:
    RemoteObject1 m_remoteObject;
};

#endif
