/**
 *  Filename: buf+msg.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#ifndef _BUF_MSG_H_
#define _BUF_MSG_H_

struct Buffer {
    Buffer() {}
    int length() { return 55; }
    char* buffer() { return 0; }
};


struct Msg {
    Msg(int i = 0) {}
};

#endif
