/**
 * @file commservice.h
 * @brief
 * The base class for asynchronous communication operations.
 *
 * It defines an array of pointers to async_operation_base objects.
 * An index to the array (used as circular buffer) is passed from the 
 * function that initiates an asynchronous operation, to the completion handler 
 * (a lambda that is used as callback function).
 *
 * The address of the async_operation_base (or a derived class) object cannot be used as argument 
 * to the lambda closure because the final address of the object is not known at the moment 
 * it would be passed to the lambda closure. 
 * Instead the index to the array element where the address will be 
 * written to, is passed to the lambda closure.
 * The move constructor of the async_operation_base object will place its address in the array at the given index.
 *
 * More flexible mechanisms are possible because the array size limits the number of operations 
 * that can be active at any moment: its size may be too big or too small.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMSERVICE_H_
#define _COMMSERVICE_H_

#include <string>
#include <chrono>
#include <assert.h>

#include "async_operation.h"

namespace corolib
{
    class async_operation_base;

    struct async_operation_info {
        async_operation_base* async_operation;
        std::chrono::high_resolution_clock::time_point start;
    };

    class CommService {
        friend class async_operation_base;
    public:
        virtual std::string get_result() { return "empty";  }

        /**
         * @brief get_free_index iterates over the array (used in a circular way)
         * for a maximum of NROPERATIONS / 2 until a free entry is found. 
         * We assume that half of the entries can be reserved for a longer time.
         * These entries have to be skipped.
         * @return index to the first free index or -1 if none is founc
         */
        int get_free_index();

        int get_free_index_ts();

        /**
         * @brief add_entry
         * @param index
         * @param op
         * @param timestamp
         *
         */
        void add_entry(int index, async_operation_base* op, bool timestamp);
        
        /**
         * @brief update_entry
         * @param index
         * @param op
         * @param timestamp
         *
         */
        void update_entry(int index, async_operation_base* op, bool timestamp);
        
        /**
         * @brief completionHandler completes an operation by calling
         * set_result() and completed() on the async_operation object.
         * Uses m_async_operations.
         * @param idx is the index into m_async_operations
         * @param in is the result of the operation
         *
         */ 
        template<typename TYPE>
        void completionHandler(int idx, TYPE in)
        {
            print(PRI2, "%p: CommService::completionHandler(idx = %d, in)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
			
			completionHandler<TYPE>(om_async_operation, in);
        }
		
        template<typename TYPE>
        void completionHandler(async_operation_base* om_async_operation, TYPE in)
        {
            print(PRI2, "%p: CommService::completionHandler(om_async_operation = %p)\n", this, om_async_operation);

            async_operation<TYPE>* om_async_operation_t =
                dynamic_cast<async_operation<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: CommService::completionHandler(om_async_operation = %p): om_async_operation_t->set_result(in)\n", this, om_async_operation_t);
                om_async_operation_t->set_result(in);
                print(PRI2, "%p: CommService::completionHandler(om_async_operation = %p): om_async_operation_t->completed()\n", this, om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: CommService::completionHandler(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation_t);
            }
        }

         /**
          * @brief completionHandler_v(oid) completes an operation by calling
          * completed() on the async_operation object.
          * Uses m_async_operations.
          * @param idx is the index into m_async_operations
          *
          */ 
        void completionHandler_v(int idx)
        {
            print(PRI2, "%p: CommService::completionHandler_v(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
			
			completionHandler_v(om_async_operation);
        }
		
        void completionHandler_v(async_operation_base* om_async_operation)
        {
            print(PRI2, "%p: CommService::completionHandler_v(om_async_operation = %p)\n", this, om_async_operation);

            if (om_async_operation)
            {
                print(PRI2, "%p: CommService::completionHandler_v(om_async_operation = %p): om_async_operation->completed()\n", this, om_async_operation);
                om_async_operation->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: CommService::completionHandler_v(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation);
            }
        }

         /**
          * @brief completionHandler_ts (timestamp) completes an operation by calling
          * set_result() and completed() on the async_operation object.
          * Uses m_async_operation_info.
          * @param idx is the index into m_async_operation_info.
          * @param start_time is the time from the m_async_operation_info element at the moment
          * the async_operation was placed at this element.
          * Value passed via the lambda closure.
          * @param in is the result of the operation.
          *
          */ 
        template<typename TYPE>
        void completionHandler_ts(const int idx, std::chrono::high_resolution_clock::time_point start_time, TYPE in)
        {
            print(PRI2, "%p: CommService::completionHandler_ts(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operation_info[idx].async_operation;
            async_operation<TYPE>* om_async_operation_t =
                dynamic_cast<async_operation<TYPE>*>(om_async_operation);

            print(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t = %p\n", this, idx, om_async_operation_t);

            if (om_async_operation_t)
            {
                if (m_async_operation_info[idx].start == start_time)
                {
                    print(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t->set_result(in)\n", this, idx);
                    om_async_operation_t->set_result(in);
                    print(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t->completed()\n", this, idx);
                    om_async_operation_t->completed();
                }
                else
                {
                    print(PRI1, "%p: CommService::completionHandler_ts(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: CommService::completionHandler_ts(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        }

         /**
          * @brief completionHandler_ts_v (timestamp_void) completes an operation by calling
          * set_result() and completed() on the async_operation object.
          * Uses m_async_operation_info.
          * @param idx is the index into m_async_operation_info.
          * @param start_time is the time from the m_async_operation_info element at the moment
          * the async_operation was placed at this element.
          * Value passed via the lambda closure.
          *
          */ 
        void completionHandler_ts_v(int idx, std::chrono::high_resolution_clock::time_point start_time)
        {
            print(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operation_info[idx].async_operation;
            //async_operation<void>* om_async_operation_t =
            //    dynamic_cast<async_operation<void>*>(om_async_operation);

            print(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): om_async_operation = %p\n", this, idx, om_async_operation);

            if (om_async_operation)
            {
                if (m_async_operation_info[idx].start == start_time)
                {
                    print(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): om_async_operation->completed()\n", this, idx);
                    om_async_operation->completed();
                }
                else
                {
                    print(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: CommService::completionHandler_ts_v(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        }

    protected:
        static const int NROPERATIONS = 128;    // use 2^N (a power of 2)

        CommService();

        virtual ~CommService();

        // Old implementation
        // Still used by many examples. Should be faded out.
        int m_index;
        async_operation_base* m_async_operations[NROPERATIONS];

        // New implementation
        // Already used by the Boost examples.
        // Will replace old implementation in the longer term.
        int m_index_ts;
        async_operation_info m_async_operation_info[NROPERATIONS];
    };
}

#endif
