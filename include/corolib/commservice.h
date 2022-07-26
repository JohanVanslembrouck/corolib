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
#include <assert.h>

namespace corolib
{
    class async_operation_base;

    class CommService {
        friend class async_operation_base;
    public:
        virtual std::string get_result() { return "empty";  }

#if 0
        int get_free_index0()
        {
            m_index = (m_index + 1) & (NROPERATIONS - 1);
            assert(m_async_operations[m_index] == nullptr);
            return m_index;
        }
#endif

        /**
         * @brief get_free_index iterates over the array (used in a circular way)
		 * for a maximum of NROPERATIONS / 2 until a free entry is found. 
		 * We assume that half of the entries can be reserved for a longer time.
		 * These entries have to be skipped.
         * @return index to the first free index or -1 if none is founc
         */
        int get_free_index()
        {
            for (int i = 0; i < NROPERATIONS / 2; i++)
            {
                m_index = (m_index + 1) & (NROPERATIONS - 1);
                if (m_async_operations[m_index] == nullptr)
                {
                    // Found free entry
                    return m_index;
                }
            }
            // No more free entries
            assert(m_async_operations[m_index] == nullptr);
            return -1;
        }

    protected:
        static const int NROPERATIONS = 128;    // use 2^N (a power of 2)

        CommService()
            : m_index(-1)
        {
            for (int i = 0; i < NROPERATIONS; i++)
                m_async_operations[i] = nullptr;
        }

        virtual ~CommService() {}

        int m_index;
        async_operation_base* m_async_operations[NROPERATIONS];
    };
}

#endif
