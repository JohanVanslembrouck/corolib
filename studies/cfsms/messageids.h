/**
 * @file messageids.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

enum class MessageId
{
    NullMsg = 0,
    
    // Messages for CFSM1
    Message001_req = 1,
    Message001_resp = -1,
    Message002_req = 2,
    Message002_resp = -2,

    // Messages for CFSM2
    Message101_req = 101,
    Message101_resp = -101,
    Message102_req = 102,
    Message102_resp = -102,
    Message103_req = 103,
    Message103_resp = -103,
    Message104_req = 104,
    Message104resp = -104
};

template <typename Enumeration>
auto as_integer(Enumeration const value)
-> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}
