/**
 * @file p1000-cfsm2.h
 * @brief
 * Contains a simple CFSM that just responds to the messages sent by any of the CFSM1 variants.
 * 
 * @author Johan Vanslembrouck
 */

#pragma once

#include <stdio.h>

#include "messageids.h"
#include "eventqueue.h"

class CFSM2
{
public:
    CFSM2(EventQueue<MessageId>& message_queue2,
          EventQueue<MessageId>& message_queue1)
        : m_message_queue2(message_queue2)
        , m_message_queue1(message_queue1)
    {
    }

    bool process_message()
    {
        MessageId msgid2 = m_message_queue2.get().value_or(MessageId::NullMsg);
        printf("msgid2 = %d\n", as_integer(msgid2));
        if (msgid2 == MessageId::NullMsg)
            return false;
        switch (msgid2) {
        case MessageId::Message101_req:
            m_message_queue1.push(MessageId::Message101_resp);
            break;
        case MessageId::Message102_req:
            m_message_queue1.push(MessageId::Message102_resp);
            break;
        default:
            break;
        }
        return true;
    }

private:
    EventQueue<MessageId>& m_message_queue2;
    EventQueue<MessageId>& m_message_queue1;
};
