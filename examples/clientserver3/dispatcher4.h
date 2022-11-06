/**
 * @file dispatcher4.h
 * @brief dispatches a request string onto a registered operation
 * This implementation is closest to one in dispatcher.h
 * because only one functor is registered in the dispatch table.
 * The registered functor now returns an async_task<int> instead of void
 * and the function dispatch uses co_await instead of a function call.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _DISPATCHER4_H_
#define _DISPATCHER4_H_

#include <functional>

#include <corolib/commservice.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/print.h>

using handleRequest =
    std::function<async_task<int>(std::string)>;

struct dispatch_table
{
    std::string     str;
    handleRequest   op;
};

class Dispatcher : public CommService
{
public:
    Dispatcher()
    {
    }
    
    /**
     * @brief registerFunctor registers a handleRequest functor with a string in a lookup table.
     * registorFunctor does not check if the string already occurs in the table,
     * but simply adds it at the end of the table.
     * @param tx is the string to associate the handleRequest with
     * @param op is the handleRequest functor to associate with the string
     */
    void registerFunctor(std::string tx, handleRequest op)
    {
        print(PRI1, "registerFunctor(%s, op)\n", tx.c_str());        
        int index = get_free_index();
        async_operation<std::string> ret{ this, index };
    
        m_dispatch_table[index].str = tx;
        m_dispatch_table[index].op = op;
    }

    /**
     * @brief getHeader retrieves the header identifying the operation from a string.
     * The header is the substring preceding the first : in the string.
     * @param str is the string to retrieve the header for,
     * @return the header found in the string or the empty string if not present
     */
    std::string getHeader(std::string str)
    {
        while (str.size())
        {
            unsigned long index = str.find(':');
            if (index != std::string::npos)
            {
                return (str.substr(0, index));
            }
        }
        return "";
    }

    /**
     * @brief dispatch takes a string and calls (dispatches) the corresponding functor.
     * Function dispatch first retrieves the header from the string.
     * It then looks up the header in the dispatch table and calls the registered functor.
     * @param str is the input string
     */
    async_task<int> dispatch(std::string str)
    {
        print(PRI2, "Dispatcher::dispatch(<%s>), m_index = %d\n", str.c_str(), m_index);
        
        std::string header = getHeader(str);

        for (int i = 0; i < m_index+1; i++)
        {
            print(PRI2, "Dispatcher::dispatch(): m_dispatch_table[%d].str = <%s>\n", i, m_dispatch_table[i].str.c_str());
            if (m_dispatch_table[i].str.compare(header) == 0)
            {
                print(PRI1, "Dispatcher::dispatch(): found match at index %d\n", i);
                async_task<int> t = m_dispatch_table[i].op(str);
                co_await t;
                break;
            }
        }
        co_return 0;
    }

protected:
    dispatch_table m_dispatch_table[NROPERATIONS];
};

#endif
