/**
 * @file handler.h
 * @brief
 * Handler_impl is the base class for interface-specific handlers
 * that are "generated" from an IDL definition.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _HANDLER_H_
#define _HANDLER_H_

namespace CORBA
{
    using PollerID = int;

    class Handler_impl
    {
    public:
        Handler_impl() {};
        virtual ~Handler_impl() {}
        virtual void operator()() {}
    };

    using Handler_impl_ptr = Handler_impl*;
}

#endif
