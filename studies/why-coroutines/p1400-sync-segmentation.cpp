/**
 * @file p1400-sync-segmentation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1400.h"

class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
		: m_remoteObjImpl(remoteObjImpl)
	{
	}

    Msg op1(Msg msg)
    {
        // Write part
        Buffer writebuffer;
        printf("RemoteObject1::op1()\n");
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        int buflength = writebuffer.length();
        for (int offset = 0; offset < buflength; offset += SEGMENT_LENGTH)
        {
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            printf("RemoteObject1::op1(): calling write_segment: offset = %d\n", offset);
            m_remoteObjImpl.write_segment(writebuffer.buffer(), offset, bytestowrite);
        }
        
        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        m_remoteObjImpl.init();
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1::op1(): calling read_segment: offset = %d\n", offset);
            completed = m_remoteObjImpl.read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        return res;
    }
protected:
	RemoteObjectImpl& m_remoteObjImpl;
};


RemoteObjectImpl remoteObjImpl;
RemoteObject1 remoteObj1{remoteObjImpl};

class Class01
{
public:
    void function1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                Msg res = remoteObj1.op1(msg);
                (void)res;
                // Do something with msg
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    class01.function1();
    //class01.function1();
    return 0;
}
