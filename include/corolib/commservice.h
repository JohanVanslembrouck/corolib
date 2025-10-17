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
 * @author Johan Vanslembrouck
 */

#ifndef _COMMSERVICE_H_
#define _COMMSERVICE_H_

#include <string>
#include <chrono>
#include <memory>
#include <assert.h>

#include "print.h"
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

        async_operation_base* get_async_operation(int idx)
        {
            if (m_async_operation_info[idx].async_operation == &reserved)
                return nullptr;
            return m_async_operation_info[idx].async_operation;
        }

        async_operation_info* get_async_operation_info(int idx)
        {
            return &m_async_operation_info[idx];
        }

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
            clprint(PRI2, "%p: CommService::completionHandler(idx = %d, in)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            
            completionHandler<TYPE>(om_async_operation, in);
        }
        
        template<typename TYPE>
        void completionHandler(async_operation_base* om_async_operation, TYPE in)
        {
            clprint(PRI2, "%p: CommService::completionHandler(om_async_operation = %p)\n", this, om_async_operation);

            async_operation<TYPE>* om_async_operation_t =
                static_cast<async_operation<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "%p: CommService::completionHandler(om_async_operation = %p): om_async_operation_t->set_result(in)\n", this, om_async_operation_t);
                om_async_operation_t->set_result(in);
                clprint(PRI2, "%p: CommService::completionHandler(om_async_operation = %p): om_async_operation_t->completed()\n", this, om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation_t);
            }
        }

        template<typename TYPE>
        void completionHandler_rmc(int idx, TYPE in)
        {
            clprint(PRI2, "%p: CommService::completionHandler_rmc(idx = %d, in)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);

            completionHandler_rmc<TYPE>(om_async_operation, in);
        }

        template<typename TYPE>
        void completionHandler_rmc(async_operation_base* om_async_operation, TYPE in)
        {
            clprint(PRI2, "%p: CommService::completionHandler_rmc(om_async_operation = %p)\n", this, om_async_operation);

            async_operation_rmc<TYPE>* om_async_operation_t =
                static_cast<async_operation_rmc<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "%p: CommService::completionHandler_rmc(om_async_operation = %p): om_async_operation_t->set_result(in)\n", this, om_async_operation_t);
                om_async_operation_t->set_result(in);
                clprint(PRI2, "%p: CommService::completionHandler_rmc(om_async_operation = %p): om_async_operation_t->completed()\n", this, om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_rmc(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation_t);
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
            clprint(PRI2, "%p: CommService::completionHandler_v(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            
            completionHandler_v(om_async_operation);
        }
        
        void completionHandler_v(async_operation_base* om_async_operation)
        {
            clprint(PRI2, "%p: CommService::completionHandler_v(om_async_operation = %p)\n", this, om_async_operation);

            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "%p: CommService::completionHandler_v(om_async_operation = %p): om_async_operation_t->completed()\n", this, om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_v(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation);
            }
        }

        void completionHandler_v_rmc(int idx)
        {
            clprint(PRI2, "%p: CommService::completionHandler_v_rmc(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);

            completionHandler_v_rmc(om_async_operation);
        }

        void completionHandler_v_rmc(async_operation_base* om_async_operation)
        {
            clprint(PRI2, "%p: CommService::completionHandler_v_rmc(om_async_operation = %p)\n", this, om_async_operation);

            async_operation_rmc<void>* om_async_operation_t =
                static_cast<async_operation_rmc<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                clprint(PRI2, "%p: CommService::completionHandler_v_rmc(om_async_operation = %p): om_async_operation_t->completed()\n", this, om_async_operation_t);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_v_rmc(om_async_operation = %p): Warning: om_async_operation_t == nullptr\n", this, om_async_operation);
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
            clprint(PRI2, "%p: CommService::completionHandler_ts(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            async_operation_base* om_async_operation = info->async_operation;
            async_operation<TYPE>* om_async_operation_t =
                static_cast<async_operation<TYPE>*>(om_async_operation);

            clprint(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t = %p\n", this, idx, om_async_operation_t);

            if (om_async_operation_t)
            {
                if (info->start == start_time)
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t->set_result(in)\n", this, idx);
                    om_async_operation_t->set_result(in);
                    clprint(PRI2, "%p: CommService::completionHandler_ts(idx = %d): om_async_operation_t->completed()\n", this, idx);
                    om_async_operation_t->completed();
                }
                else
                {
                    clprint(PRI1, "%p: CommService::completionHandler_ts(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_ts(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        }

        template<typename TYPE>
        void completionHandler_ts_rmc(const int idx, std::chrono::high_resolution_clock::time_point start_time, TYPE in)
        {
            clprint(PRI2, "%p: CommService::completionHandler_ts_rmc(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            async_operation_base* om_async_operation = info->async_operation;
            async_operation_rmc<TYPE>* om_async_operation_t =
                static_cast<async_operation_rmc<TYPE>*>(om_async_operation);

            clprint(PRI2, "%p: CommService::completionHandler_ts_rmc(idx = %d): om_async_operation_t = %p\n", this, idx, om_async_operation_t);

            if (om_async_operation_t)
            {
                if (info->start == start_time)
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts_rmc(idx = %d): om_async_operation_t->set_result(in)\n", this, idx);
                    om_async_operation_t->set_result(in);
                    clprint(PRI2, "%p: CommService::completionHandler_ts_rmc(idx = %d): om_async_operation_t->completed()\n", this, idx);
                    om_async_operation_t->completed();
                }
                else
                {
                    clprint(PRI1, "%p: CommService::completionHandler_ts_rmc(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_ts_rmc(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
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
            clprint(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            async_operation_base* om_async_operation = info->async_operation;
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            clprint(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): om_async_operation = %p\n", this, idx, om_async_operation);

            if (om_async_operation_t)
            {
                if (info->start == start_time)
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): om_async_operation_t->completed()\n", this, idx);
                    om_async_operation_t->completed();
                }
                else
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts_v(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_ts_v(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        }

        void completionHandler_ts_v_rmc(int idx, std::chrono::high_resolution_clock::time_point start_time)
        {
            clprint(PRI2, "%p: CommService::completionHandler_ts_v_rmc(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            async_operation_base* om_async_operation = info->async_operation;
            async_operation_rmc<void>* om_async_operation_t =
                static_cast<async_operation_rmc<void>*>(om_async_operation);

            clprint(PRI2, "%p: CommService::completionHandler_ts_v_rmc(idx = %d): om_async_operation = %p\n", this, idx, om_async_operation);

            if (om_async_operation_t)
            {
                if (info->start == start_time)
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts_v_rmc(idx = %d): om_async_operation_t->completed()\n", this, idx);
                    om_async_operation_t->completed();
                }
                else
                {
                    clprint(PRI2, "%p: CommService::completionHandler_ts_v_rmc(idx = %d): Warning: entry already taken by other operation\n", this, idx);
                }
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                clprint(PRI1, "%p: CommService::completionHandler_ts_v_rmc(idx = %d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        }

        int get_table_size() { return m_index; }

        constexpr int get_table_entries() { return NROPERATIONS; }

    protected:
        static const int NROPERATIONS = 128;    // use 2^N (a power of 2)

        CommService(int nr_operations = NROPERATIONS);
        virtual ~CommService();

    private:
        static async_operation_base reserved;

        std::shared_ptr<async_operation_info[]> m_async_operation_info;
        int m_nr_operations;
        int m_index;
    };
}

#endif
