/**
 * @file dispatcher3.h
 * @brief dispatches a request string onto a registered operation
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _DISPATCHER3_H_
#define _DISPATCHER3_H_

#include <functional>

#include <corolib/commservice.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/print.h>

using handleRequest =
    std::function<async_task<int>(std::string)>;
using handleRequest2 =
    std::function<async_task<int>(std::string, int)>;

struct dispatch_table
{
    std::string     str;
    handleRequest   op;
    handleRequest2  op2;
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
     * In contrast to registerFunctor in dispatcher.h, this registerFunctor
     * also enters a self-created lambda in the dispatcher table.
     * In contast to registerFunctor in displatcher2.h, the signature of 
     * the lambda is different and an explicit declaration of the return type is necessary.
     * This lambda calls the handleRequest functor.
     * @param tx is the string to associate the handleRequest with
     * @param op is the handleRequest functor to associate with the string
     */
    async_operation<std::string> registerFunctor(std::string tx, handleRequest op)
    {
        print(PRI1, "registerFunctor(%s, op)\n", tx.c_str());
        
        int index = get_free_index();
        async_operation<std::string> ret{ this, index };
    
        m_dispatch_table[index].str = tx;
        m_dispatch_table[index].op = op;
        m_dispatch_table[index].op2 = [this](std::string str, int idx) -> async_task<int>
        {
            print(PRI1, "lambda: idx = %d, str = <%s>\n", idx, str.c_str());
            async_task<int> t = m_dispatch_table[idx].op(str);
            co_await t;

            async_operation<std::string>* om_async_operation = 
                dynamic_cast<async_operation<std::string>*>(m_async_operations[idx]);
            if (om_async_operation)
            {
                om_async_operation->set_result(str);
                om_async_operation->completed();
            }

            co_return 0;
        };
        return ret;
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
     * It then looks up the header in the dispatch table and calls the registered lambda.
     * dispatch only calls the lambda that registerFunctor created before.
     * @param str is the input string
     */
    void dispatch(std::string str)
    {
        print(PRI2, "Dispatcher::dispatch(<%s>), m_index = %d\n", str.c_str(), m_index);
        
        std::string header = getHeader(str);

        for (int i = 0; i < m_index+1; i++)
        {
            print(PRI2, "Dispatcher::dispatch(): m_dispatch_table[%d].str = <%s>\n", i, m_dispatch_table[i].str.c_str());
            if (m_dispatch_table[i].str.compare(header) == 0)
            {
                print(PRI1, "Dispatcher::dispatch(): found match at index %d\n", i);
                m_dispatch_table[m_index].op2(str, i);
                break;
            }
        }
    }

protected:
    dispatch_table m_dispatch_table[NROPERATIONS];
};

#endif
