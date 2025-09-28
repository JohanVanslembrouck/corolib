/**
 * @file messagemapper.h
 * @brief
 * Associates message ids with the functor to execute.
 * 
 * @author Johan Vanslembrouck
 */

#pragma once

#include <stdio.h>
#include <functional>

class MessageMapper
{
private:
    struct MsgMap
    {
        MessageId msgId;
        std::function<void(int)> function;
    };

public:
    MessageMapper()
    {
        for (int i = 0; i < NR_ENTRIES; ++i)
        {
            msgmap[i].msgId = MessageId::NullMsg;
        }
    }

    void add(MessageId msgId, std::function<void(int)>&& function)
    {
        for (int i = 0; i < NR_ENTRIES; ++i)
        {
            if (msgmap[i].msgId == MessageId::NullMsg) {
                msgmap[i].msgId = msgId;
                msgmap[i].function = std::move(function);
                break;
            }
        }
    }

    void find(MessageId msgId)
    {
        // printf("find(%d)\n", as_integer(msgId));
        for (int i = 0; i < NR_ENTRIES; ++i)
        {
            if (msgmap[i].msgId == msgId)
            {
                msgmap[i].function(as_integer(msgId));
                msgmap[i].msgId = MessageId::NullMsg;
                break;
            }
        }
    }

private:
    const static int NR_ENTRIES = 32;
    MsgMap msgmap[NR_ENTRIES];
};
