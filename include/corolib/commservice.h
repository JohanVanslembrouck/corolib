/**
 * @file commservice.h
 * @brief
 * The base class for asynchronous communication operations.
 *
 * It defines an array of pointers to async_operations.
 * An index to the array (used as circular buffer) is passed from the 
 * function that initiates the asynchronous operation, to the completion handler 
 * (a lambda that is used as callback function).
 * The address of the async_operation_base (or a derived class) object cannot be used as argument in the lambda closure.
 * The constructor of the async_operation_base object will place its address in the array at the given index.
 *
 * More flexible mechanisms are possible because the array size limits the number of operations 
 * that can be active at any moment: its size may be too big or too small.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMSERVICE_H_
#define _COMMSERVICE_H_

#include <string>

namespace corolib
{
	class async_operation_base;

	class CommService {
		friend class async_operation_base;
	public:
		virtual std::string get_result() { return "empty";  }

	protected:
		static const int NROPERATIONS = 128;	// use 2^N

		CommService()
			: index(-1)
		{
			for (int i = 0; i < NROPERATIONS; i++)
				m_async_operations[i] = nullptr;
		}

		virtual ~CommService() {}

		int index;
		async_operation_base* m_async_operations[NROPERATIONS];
	};
}

#endif
