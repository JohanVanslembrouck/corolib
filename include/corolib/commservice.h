/**
 * @file commservice.h
 * @brief
 * Defines a synonym for async_operation_register.
 * CommService has been used in many examples since the start of the corolib project in 2020,
 * but it is a name that does not reflect its content (anymore).
 * 
 * To avoid making changes to these examples, this file simply defines a synonym for async_operation_register.
 * The original content of commservice.h is now in async_operation_register.h.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _COMMSERVICE_H_
#define _COMMSERVICE_H_

#include "async_operation_register.h"

namespace corolib
{
    using CommService = async_operation_register;
}

#endif
