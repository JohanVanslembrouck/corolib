/**
 * @file buf+msg.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _BUF_MSG_H_
#define _BUF_MSG_H_

struct Buffer
{
    Buffer() {}
    int length() { return 55; }		// Fixed buffer length for write buffers
    char* buffer() { return 0; }
};


struct Msg
{
    Msg(int i = 0) {}
};


const int INBUFFER_LENGTH = 35;		// Fixed buffer length for read buffers
const int SEGMENT_LENGTH = 10;		// Maximum number of bytes we can read or write
const int MAX_MSG_LENGTH = 5;
const int NR_MSGS_TO_SEND = 10;

#endif
