/**
 * @file co-less07.cpp
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
    void sendc_write_segment(char* p, int offset, lambda3 l) {
        eventQueue.push(l);
    }

    bool read_segment(char* p, int offset, int segment_length) {
        printf("RemoteObjectImpl::read_segment(p, offset = %d, segment_length = %d)\n", offset, segment_length);
        return (offset > segment_length);
    }
    void sendc_read_segment(char* p, int offset, int segment_length, lambda3 l) {
        eventQueue.push(l);
    }
};

RemoteObjectImpl remoteObjImpl;

// -----------------------------------------------------------------------------

struct RemoteObject3 {
    int offset = 0;
    Buffer buf;
    bool completed = false;
    Buffer buf2;
    lambda1 l;

    void sendc_op1(int in11, int in12, lambda1 op1cb) {
        printf("RemoteObject3::sendc_op1(): calling write_segment\n");
        l = op1cb;
        // Marshall in11 and in12 into buf
        remoteObjImpl.sendc_write_segment(buf.buffer(), offset,
            [this]() { this->op1a(); });
    }

    void op1a() {
        printf("RemoteObject3::op1a()\n");
        if (offset < buf.length()) {
            printf("RemoteObject3::op1a(): calling sendc_write_segment\n");
            remoteObjImpl.sendc_write_segment(buf.buffer(), offset, 
                [this]() { this->op1a(); });
            offset += segment_length;
        }
        else {
            offset = 0;
            printf("RemoteObject3::op1a(): calling sendc_read_segment\n");
            remoteObjImpl.sendc_read_segment(buf2.buffer(), offset, segment_length, 
                                                            [this]() { this->op1b(); });
        }
    }

    void op1b() {
        printf("RemoteObject3::sendc_op1b(): calling sendc_read_segment\n");
        if (offset < segment_length) {
            remoteObjImpl.sendc_read_segment(buf2.buffer(), offset, segment_length, 
                                                            [this]() { this->op1b(); });
            offset += segment_length;
        }
        else {
            // Unmarshall out11, out12 and ret1 from buf2
            l(out11, out12, ret1);
        }
    }

    void callback(int out11, int out12, int ret1) {
        printf("RemoteObject3::callback()\n");
    }
};

RemoteObject3 remoteObject3;

int main() {
    printf("main();\n");
    connect(event1, []() { remoteObject3.sendc_op1(in11, in12,
                            [](int out11, int out12, int ret1) { remoteObject3.callback(out11, out12, ret1); });
        });
    connect(event2, []() { remoteObject3.sendc_op1(in11, in12,
                            [](int out11, int out12, int ret1) { remoteObject3.callback(out11, out12, ret1); });
        });
    eventQueue.run();
    return 0;
}

