/**
 * @file p1000-cfsms.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "eventqueue.h"
#include "messageids.h"

#include "p1000-cfsm1.h"
#include "p1000-cfsm2.h"

int main()
{
    EventQueue<MessageId> message_queue1;
    EventQueue<MessageId> message_queue2;

    CFSM1 cfsm1{ message_queue1, message_queue2 };
    CFSM2 cfsm2{ message_queue2, message_queue1 };
    
    // Kick-off the system by sending a message to cfsm1
    message_queue1.push(MessageId::Message001_req);

    while (true)
    {
        if (!cfsm1.process_message())
            break;
        if (!cfsm2.process_message())
            break;
    } 
	return 0;
}
