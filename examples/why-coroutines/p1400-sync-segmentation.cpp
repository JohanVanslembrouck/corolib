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
    void write_segment(char* p, int offset) {
        printf("RemoteObjectImpl::write_segment(p, offset = %d)\n", offset);
    }
    void sendc_write_segment(char* p, int offset, lambda_void_t l) {
        eventQueue.push(l);
    }

    bool read_segment(char* p, int offset, int segment_length) {
        printf("RemoteObjectImpl::read_segment(p, offset = %d, segment_length = %d)\n", offset, segment_length);
        return (offset > segment_length);
    }
    void sendc_read_segment(char* p, int offset, int segment_length, lambda_void_t l) {
        eventQueue.push(l);
    }
};

RemoteObjectImpl remoteObjImpl;

struct RemoteObject2 {
    int op1(int in11, int in12, int& out11, int& out12) {
        Buffer buf;
        printf("RemoteObject2::op1()\n");
        // Marshall in11 and in12 into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject2::op1(): calling write_segment\n");
            remoteObjImpl.write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject2::op1(): calling read_segment\n");
            completed = remoteObjImpl.read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall out11, out12 and ret1 from buf2
        return gret1;
    }
};

RemoteObject2 remoteObject2;

int main() {
    printf("main();\n");
    connect(event1, []() { remoteObject2.op1(gin11, gin12, gout11, gout12); });
    connect(event2, []() { remoteObject2.op1(gin11, gin12, gout11, gout12); });
    eventQueue.run();
    return 0;
}
