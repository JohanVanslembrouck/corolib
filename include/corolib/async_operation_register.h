/**
 * @file async_operation_register.h
 * @brief
 * async_operation_register is a register (inventory) of pointers to async_operation_base derived class objects 
 * (in short, async_operation objects).
 *
 * Upon creation, an async_operation object registers its address in a free entry.
 * When an async_operation object goes out of scope, it removes its address and replaces it with the nullptr.
 * When an async_operation object is moved, the address of the original object is replaced with the address of the destination object.
 * 
 * Instead of passing a pointer to an async_operation object to a function (typically, a callback function that will be called
 * at a later time), we now pass the index to the entry where the object has stored its address.
 * 
 * At the moment the callback function is called, we can now deal with the following cases:
 * - When the async_operation object is still at its original location, the callback function finds and writes to this object.
 * - When the async_operation object has gone out of scope, the callback function finds a nullptr and will not try to write to the object.
 * - When the async_operation object has been moved, the callback function finds a pointer to the current object and writes to this object.
 * 
 * The reader is referred to studies/rvo for the reasons for a study of these problems.
 * 
 * Application classes defining functions returning an async_operation object
 * typically inherit from async_operation_register or define an async_operation_register data member.
 * 
 * class async_operation_register is currently implemented using an array of pointers to async_operation_base objects.
 * The size of the array is NROPERATIONS. This size should be larger than the maximum number of async_operation objects
 * that can be stored simultaneously at any one time.
 * More flexible mechanisms are possible because the array size limits the number of operations 
 * that can be active at any moment: its size may be too big or too small.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _ASYNC_OPERATION_REGISTER_H_
#define _ASYNC_OPERATION_REGISTER_H_

#include <string>
#include <chrono>
#include <memory>

#include "print.h"
#include "async_operation.h"
#include "async_operation_access.h"

namespace corolib
{
    struct async_operation_info
    {
        async_operation_base* async_operation;
        std::chrono::high_resolution_clock::time_point start;
    };

    class async_operation_register
    {
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
            clprint(PRI2, "%p: async_operation_register::completionHandler(idx = %d, in)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            
            async_operation_access::completionHandler<TYPE>(om_async_operation, in);
        }

        template<typename TYPE>
        void completionHandler_rmc(int idx, TYPE in)
        {
            clprint(PRI2, "%p: async_operation_register::completionHandler_rmc(idx = %d, in)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);

            async_operation_access::completionHandler_rmc<TYPE>(om_async_operation, in);
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
            clprint(PRI2, "%p: async_operation_register::completionHandler_v(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            
            async_operation_access::completionHandler_v(om_async_operation);
        }

        void completionHandler_v_rmc(int idx)
        {
            clprint(PRI2, "%p: async_operation_register::completionHandler_v_rmc(idx = %d)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);

            async_operation_access::completionHandler_v_rmc(om_async_operation);
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
            clprint(PRI2, "%p: async_operation_register::completionHandler_ts(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            if (info->start == start_time)
            {
                async_operation_access::completionHandler<TYPE>(info->async_operation, in);
            }
            else
            {
                clprint(PRI1, "async_operation_register::completionHandler_ts(): Warning: entry already taken by other operation\n");
            }
        }

        template<typename TYPE>
        void completionHandler_ts_rmc(const int idx, std::chrono::high_resolution_clock::time_point start_time, TYPE in)
        {
            clprint(PRI2, "%p: async_operation_register::completionHandler_ts_rmc(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            if (info->start == start_time)
            {
                async_operation_access::completionHandler_rmc<TYPE>(info->async_operation, in);
            }
            else
            {
                clprint(PRI1, "async_operation_register::completionHandler_ts(): Warning: entry already taken by other operation\n");
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
            clprint(PRI2, "%p: async_operation_register::completionHandler_ts_v(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            if (info->start == start_time)
            {
                async_operation_access::completionHandler_v(info->async_operation);
            }
            else
            {
                clprint(PRI1, "async_operation_register::completionHandler_ts(): Warning: entry already taken by other operation\n");
            }
        }

        void completionHandler_ts_v_rmc(int idx, std::chrono::high_resolution_clock::time_point start_time)
        {
            clprint(PRI2, "%p: async_operation_register::completionHandler_ts_v_rmc(idx = %d)\n", this, idx);

            async_operation_info* info = get_async_operation_info(idx);

            if (info->start == start_time)
            {
                async_operation_access::completionHandler_v_rmc(info->async_operation);
            }
            else
            {
                clprint(PRI1, "async_operation_register::completionHandler_ts(): Warning: entry already taken by other operation\n");
            }
        }

        int get_table_size() { return m_index; }

        constexpr int get_table_entries() { return NROPERATIONS; }

    protected:
        static const int NROPERATIONS = 128;    // use 2^N (a power of 2)

        async_operation_register(int nr_operations = NROPERATIONS);
        virtual ~async_operation_register();

    private:
        static async_operation_base reserved;

        std::shared_ptr<async_operation_info[]> m_async_operation_info;
        int m_nr_operations;
        int m_index;
    };
}

#endif
