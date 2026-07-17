/**
 * @file async_operation_access.h
 * @brief struct async_operation_access contains 4 static functions
 * that are called from completion handlers (with the same name) in async_operation_register.h
 * or that are called directly from application code.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _ASYNC_OPERATION_ACCESS_H_
#define _ASYNC_OPERATION_ACCESS_H_

#include <string>
#include <chrono>

#include "print.h"
#include "async_operation.h"

namespace corolib
{
    struct async_operation_access
    {
        template<typename TYPE>
        static void completionHandler(async_operation_base* om_async_operation, TYPE in)
        {
            clprint(PRI2, "async_operation_access::completionHandler(om_async_operation = %p)\n", om_async_operation);

            async_operation<TYPE>* om_async_operation_t =
                static_cast<async_operation<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "async_operation_access::completionHandler(om_async_operation = %p): om_async_operation_t->set_result(in)\n", om_async_operation_t);
                om_async_operation_t->set_result(in);
                clprint(PRI2, "async_operation_access::completionHandler(om_async_operation = %p): om_async_operation_t->completed()\n", om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "async_operation_access::completionHandler(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", om_async_operation_t);
            }
        }

        template<typename TYPE>
        static void completionHandler_rmc(async_operation_base* om_async_operation, TYPE in)
        {
            clprint(PRI2, "async_operation_access::completionHandler_rmc(om_async_operation = %p)\n", om_async_operation);

            async_operation_rmc<TYPE>* om_async_operation_t =
                static_cast<async_operation_rmc<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "async_operation_access::completionHandler_rmc(om_async_operation = %p): om_async_operation_t->set_result(in)\n", om_async_operation_t);
                om_async_operation_t->set_result(in);
                clprint(PRI2, "async_operation_access::completionHandler_rmc(om_async_operation = %p): om_async_operation_t->completed()\n", om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "async_operation_access::completionHandler_rmc(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", om_async_operation_t);
            }
        }

        static void completionHandler_v(async_operation_base* om_async_operation)
        {
            clprint(PRI2, "async_operation_access::completionHandler_v(om_async_operation = %p)\n", om_async_operation);

            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "async_operation_access::completionHandler_v(om_async_operation = %p): om_async_operation_t->completed()\n", om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "async_operation_access::completionHandler_v(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", om_async_operation);
            }
        }

        static void completionHandler_v_rmc(async_operation_base* om_async_operation)
        {
            clprint(PRI2, "%async_operation_access::completionHandler_v_rmc(om_async_operation = %p)\n", om_async_operation);

            async_operation_rmc<void>* om_async_operation_t =
                static_cast<async_operation_rmc<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "async_operation_access::completionHandler_v_rmc(om_async_operation = %p): om_async_operation_t->completed()\n", om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%async_operation_access::completionHandler_v_rmc(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", om_async_operation);
            }
        }
    };
}

#endif
