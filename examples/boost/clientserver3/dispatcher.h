/**
 * @file dispatcher.h
 * @brief dispatches a request string onto a registered operation
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _DISPATCHER_H_
#define _DISPATCHER_H_

#include <functional>

#include <corolib/print.h>

using handleRequest = 
    std::function<void(std::string)>;

struct dispatch_table
{
    std::string     str;
    handleRequest   op;
};

class Dispatcher
{
public:
    static const int NROPERATIONS = 128;

    Dispatcher()
        : m_index(0)
    {
        print(PRI1, "Dispatcher::Dispatcher()\n");
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
        print(PRI1, "Dispatcher::registerFunctor(%s, op)\n", tx.c_str());
        if (m_index < NROPERATIONS)
        {
            m_dispatch_table[m_index].str = tx;
            m_dispatch_table[m_index].op = op;
            m_index++;
        }
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
    void dispatch(std::string str)
    {
        print(PRI2, "Dispatcher::dispatch(str), m_index = %d, str = %s", m_index, str.c_str());

        std::string header = getHeader(str);

        for (int i = 0; i < m_index; i++)
        {
            print(PRI2, "Dispatcher::dispatch(): m_dispatch_table[%d].str = <%s>\n", i, m_dispatch_table[i].str.c_str());
            if (m_dispatch_table[i].str.compare(header) == 0)
            {
                print(PRI1, "Dispatcher::dispatch(): found match at index %d\n", i);
                m_dispatch_table[i].op(str);
                break;
            }
        }
    }
    
protected:
    dispatch_table m_dispatch_table[NROPERATIONS];
    int m_index;
};

#endif
