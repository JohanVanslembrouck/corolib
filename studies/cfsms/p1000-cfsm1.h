/**
 * @file p1000-cfsm1.h
 * @brief
 * Contains a first implementation of CFSM1 that uses a state machine implementation
 * with a double select on the state and the message id.
 * 
 * @author Johan Vanslembrouck
 */

#pragma once

#include <stdio.h>

#include "messageids.h"
#include "eventqueue.h"

class CFSM1
{
    enum class State
    {
        State1,
        State2,
        State3,
        State4,
        State5
    };

public:
    CFSM1(EventQueue<MessageId>& message_queue1,
        EventQueue<MessageId>& message_queue2)
        : m_message_queue1(message_queue1)
        , m_message_queue2(message_queue2)
    {
    }

    bool process_message()
    {
        MessageId msgid1 = m_message_queue1.get().value_or(MessageId::NullMsg);
        printf("msgid1 = %d\n", as_integer(msgid1));
        
        if (msgid1 == MessageId::NullMsg)
            return false;

        switch (m_state) {
        case State::State1:
            switch (msgid1) {
            case MessageId::Message001_req:
                m_message_queue2.push(MessageId::Message101_req);
                m_state = State::State2;
                break;
            default:
                break;
            }
            break;
        case State::State2:
            switch (msgid1) {
            case MessageId::Message101_resp:
                m_message_queue2.push(MessageId::Message102_req);
                m_state = State::State3;
                break;
            default:
                break;
            }
            break;
        case State::State3:
            switch (msgid1) {
                case MessageId::Message102_resp:
                    m_state = State::State3;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
        }

        return true;
    }

private:
    EventQueue<MessageId>& m_message_queue1;
    EventQueue<MessageId>& m_message_queue2;

    State m_state{ State::State1 };
};

