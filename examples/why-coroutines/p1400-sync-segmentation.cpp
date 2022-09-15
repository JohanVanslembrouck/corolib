/**
 * @file p1400-sync-segmentation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

class RemoteObjectImpl {
public:
    void write_segment(char* p, int offset)
    {
        printf("RemoteObjectImpl::write_segment(p, offset = %d)\n", offset);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool read_segment(char* p, int offset, int segment_length)
    {
        printf("RemoteObjectImpl::read_segment(p, offset = %d, segment_length = %d)\n", offset, segment_length);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return (offset > segment_length);
    }
};

RemoteObjectImpl remoteObjImpl;

struct RemoteObject1
{
    Msg op1(Msg msg)
    {
        Buffer buf;
        Msg res;
        printf("RemoteObject1::op1()\n");
        // Marshall msg into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject1::op1(): calling write_segment: offset = %d\n", offset);
            remoteObjImpl.write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject1::op1(): calling read_segment: offset = %d\n", offset);
            completed = remoteObjImpl.read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall Msg from buf2
        return res;
    }
};

RemoteObject1 remoteObject1;

struct Class01
{
    void function1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class01::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                Msg res = remoteObject1.op1(msg);
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    //connect(event1, []() { remoteObject1.op1(gin11, gin12, gout11, gout12); });
    //connect(event2, []() { remoteObject1.op1(gin11, gin12, gout11, gout12); });
    connect(event1, []() { class01.function1(); });
    //connect(event2, []() { class01.function1(); });
    eventQueue.run();
    return 0;
}
