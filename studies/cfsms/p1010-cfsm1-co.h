/**
 * @file p1010-cfsm1-co.h
 * @brief
 * Contains an implementation of CFSM1 using coroutines and async_task.
 * 
 * @author Johan Vanslembrouck
 */

#pragma once

#include <functional>

#include <corolib/commservice.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "eventqueue.h"
#include "messageids.h"
#include "messagemapper.h"

class CFSM1 : public CommService
{
public:
    CFSM1(EventQueue<MessageId>& message_queue1,
        EventQueue<MessageId>& message_queue2)
        : m_message_queue1(message_queue1)
        , m_message_queue2(message_queue2)
    {
    }

    void async_op(MessageId msgid1, MessageId msgid2, std::function<void(int)>&& completionHandler)
    {
        m_messagemapper.add(msgid2, std::move(completionHandler));
        m_message_queue2.push(msgid1);
    }

    async_operation<int> start_operation1()
    {
        int idx = get_free_index();
        async_operation<int> ret{ this, idx };
        async_op(MessageId::Message101_req, MessageId::Message101_resp,
            [this, idx](int i)
            {
                completionHandler<int>(idx, i);
            });
        return ret;
    }

    async_operation<int> start_operation2(int i)
    {
        int idx = get_free_index();
        async_operation<int> ret{ this, idx };
        async_op(MessageId::Message102_req, MessageId::Message102_resp,
            [this, idx](int i)
            {
                completionHandler<int>(idx, i);
            });
        return ret;
    }

    async_task<void> mainflow()
    {
        printf("mainflow(): co_await start_operation1()\n");
        co_await start_operation1();
        printf("mainflow(): co_await start_operation2(4)\n");
        co_await start_operation2(4);
        co_return;
    }

    bool process_message(async_task<void>& t1)
    {
        MessageId msgid1 = m_message_queue1.get().value_or(MessageId::NullMsg);

        printf("msgid1 = %d\n", as_integer(msgid1));

        if (msgid1 == MessageId::NullMsg)
            return false;

        if (msgid1 == MessageId::Message001_req) {
            t1 = mainflow();
        }
        else
            m_messagemapper.find(msgid1);

        return true;
    }

private:
    EventQueue<MessageId>& m_message_queue1;
    EventQueue<MessageId>& m_message_queue2;
    MessageMapper m_messagemapper;
};
