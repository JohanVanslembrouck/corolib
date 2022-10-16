/**
 * @file buf+msg.h
 * @brief Contains a very simplified buffer and message implementation.
 * Suffices for use in the synchronous / asynchronous / coroutine examples.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _BUF_MSG_H_
#define _BUF_MSG_H_

struct Buffer
{
    Buffer() {}
    int length() { return 55; }     // Fixed buffer length for write buffers
    char* buffer() { return 0; }
};

struct Msg
{
    Msg(int i = 0) {}               // i = indication for the message length, but currently ignored
};

const int INBUFFER_LENGTH = 35;     // Fixed buffer length for read buffers
const int SEGMENT_LENGTH = 10;      // Maximum number of bytes we can read or write at once
const int MAX_MSG_LENGTH = 5;       // Number of messages (to "generate") with a different length
const int NR_MSGS_TO_SEND = 10;     // Number of messages (with the same length) to send

#endif
