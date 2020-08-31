/**
 * @file dispatcher.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _DISPATCHER2_H_
#define _DISPATCHER2_H_

#include <functional>

#include "corolib/commservice.h"
#include "corolib/async_operation.h"
#include "corolib/print.h"

using handleRequest =
	std::function<void(std::string)>;
using handleRequest2 =
	std::function<void(std::string, int)> ;

struct dispatch_table
{
	std::string 	str;
	handleRequest	op;
	handleRequest2	op2;
};

class Dispatcher : public CommService
{
public:
	Dispatcher()
	{
	}
	
	async_operation_t<std::string> registerFunctor(std::string tx, handleRequest op)
	{
		print(PRI1, "registerFunctor(%s, op)\n", tx.c_str());
		
		index = (index + 1) & (NROPERATIONS - 1);
		assert(m_async_operations[index] == nullptr);
		async_operation_t<std::string> ret{ this, index };
	
		m_dispatch_table[index].str = tx;
		m_dispatch_table[index].op = op;
		m_dispatch_table[index].op2 = [this](std::string str, int idx)
		{
			print(PRI1, "lambda: idx = %d, str = <%s>\n", idx, str.c_str());
			m_dispatch_table[idx].op(str);

			async_operation_t<std::string>* om_async_operation = 
				dynamic_cast<async_operation_t<std::string>*>(m_async_operations[idx]);
			if (om_async_operation)
			{
				om_async_operation->set_result(str);
				om_async_operation->completed();
			}
		};
		return ret;
	}

	std::string getHeader(std::string str) {
		while (str.size()) {
			int index = str.find(':');
			if (index != std::string::npos) {
				return (str.substr(0, index));
			}
		}
		return "";
	}

	void dispatch(std::string str)
	{
		print(PRI2, "Dispatcher::dispatch(<%s>), index = %d\n", str.c_str(), index);
		
		std::string header = getHeader(str);

		for(int i = 0; i < index+1; i++)
		{
			print(PRI2, "Dispatcher::dispatch(): m_dispatch_table[%d].str = <%s>\n", i, m_dispatch_table[i].str.c_str());
			if (m_dispatch_table[i].str.compare(header) == 0)
			{
				print(PRI1, "Dispatcher::dispatch(): found match at index %d\n", i);
				m_dispatch_table[index].op2(str, i);
				break;
			}
		}
	}

protected:
	dispatch_table m_dispatch_table[NROPERATIONS];
};

#endif
