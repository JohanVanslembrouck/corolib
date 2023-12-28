/**
 * @file p1410-async-segmentation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1400.h"

using lambda_msg_t = typename std::function<void(Msg)>;

class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
		: m_remoteObjImpl(remoteObjImpl)
	{
	}
	
    void init()
    {
        offset = 0;
        completed = false;
    }
    
    void sendc_op1(Msg msg, lambda_msg_t op1_cb)
    {
        printf("RemoteObject1::sendc_op1(): calling write_segment\n");
        lambda = op1_cb;
        
        // Write part
        // Marshall msg into writebuffer
        // (code not present)
        // Write the first segment
        int buflength = writebuffer.length();
        int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
        m_remoteObjImpl.sendc_write_segment(writebuffer.buffer(), offset, bytestowrite,
                                            [this]() { this->handle_write_segment(); });
        offset += SEGMENT_LENGTH;
    }

    void handle_write_segment()
    {
        printf("RemoteObject1::handle_write_segment()\n");
        int buflength = writebuffer.length();
        if (offset < buflength) {
            printf("RemoteObject1::handle_write_segment(): calling sendc_write_segment\n");
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            m_remoteObjImpl.sendc_write_segment(writebuffer.buffer(), offset, bytestowrite,
                                                [this]() { this->handle_write_segment(); });
            offset += SEGMENT_LENGTH;
        }
        else {
            // Read part
            offset = 0;
            m_remoteObjImpl.init();
            printf("RemoteObject1::handle_write_segment(): calling sendc_read_segment\n");
            m_remoteObjImpl.sendc_read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH, 
                                                            [this](bool res) { this->handle_read_segment(res); });
            offset += SEGMENT_LENGTH;
        }
    }

    void handle_read_segment(bool complete)
    {
        Msg msg;
        if (!complete) {
            printf("RemoteObject1::handle_read_segment(): calling sendc_read_segment\n");
            m_remoteObjImpl.sendc_read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH, 
                                                            [this](bool res) { this->handle_read_segment(res); });
            offset += SEGMENT_LENGTH;
        }
        else {
            // Unmarshall msg from buf
            // (code not present)
            // Invoke the lambda passing the result
            lambda(msg);
        }
    }

protected:
	RemoteObjectImpl& m_remoteObjImpl;
	
private:
    int offset = 0;
    Buffer writebuffer;
    bool completed = false;
    Buffer readbuffer;
    lambda_msg_t lambda;
};


RemoteObjectImpl remoteObjImpl;
RemoteObject1 remoteObj1{remoteObjImpl};

class Class01
{
public:
    void function1()
    {
		counter = 0;
        printf("Class01::function1(): counter = %d\n", counter);
        i = j = 0;
        
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.init();
        remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
    }

    void function1a(Msg msgout)
    {
        // Do something with msgout
        printf("Class01::function1a(Msg): counter = %d\n", counter);
        if (j < NR_MSGS_TO_SEND) {
            remoteObj1.init();
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
            j++;
            counter++;
        }
        else {
            // End of inner loop
            j = 0;
            i++;
            if (i < MAX_MSG_LENGTH) {
                msg = Msg(i);
                remoteObj1.init();
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
                j++;
                counter++;
            }
            else {
                // End of inner and outer loop
                elapsed_time = get_current_time() - start_time;
            }
        }
    }
    
private:
    int i, j;
    Msg msg;
    int counter;
};

Class01 class01;
Msg gmsg1;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01.function1();
    eventQueue.run();
    return 0;
}
