
/**
 * @file p1010-cfsms-co.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "eventqueue.h"
#include "messageids.h"

#include "p1010-cfsm1-co.h"
#include "p1000-cfsm2.h"

int main()
{
    EventQueue<MessageId> message_queue1;
    EventQueue<MessageId> message_queue2;

    CFSM1 cfsm1{ message_queue1, message_queue2 };
    CFSM2 cfsm2{ message_queue2, message_queue1 };

    // Kick-off the system by sending a message to cfsm1
    message_queue1.push(MessageId::Message001_req);

    async_task<void> t1;
    while (true)
    {
        if (!cfsm1.process_message(t1))
            break;
        if (!cfsm2.process_message())
            break;
    }
    t1.wait();

    return 0;
}
