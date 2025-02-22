/**
 * @file buf+msg.h
 * @brief Contains a very simplified buffer and message implementation.
 * Suffices for use in the synchronous / asynchronous / coroutine examples.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _BUF_MSG_H_
#define _BUF_MSG_H_

const int OUTBUFFER_LENGTH = 55;    // Fixed buffer length for write buffers
const int INBUFFER_LENGTH = 35;     // Fixed buffer length for read buffers
const int SEGMENT_LENGTH = 10;      // Maximum number of bytes we can read or write at once
const int MAX_MSG_LENGTH = 5;       // Number of messages (to "generate") with a different length
const int NR_MSGS_TO_SEND = 10;     // Number of messages (with the same length) to send

struct Buffer
{
    Buffer() {}
    int length() { return OUTBUFFER_LENGTH; }
    char* buffer() { return 0; }
};

struct Msg
{
    Msg(int length = 0)
        : m_length(length)
    {}

    int m_length;
};

#endif
