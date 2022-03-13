/**
 * @file dispatcher.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
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
    handleRequest    op;
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
        print(PRI2, "Dispatcher::dispatch(<%s>), m_index = %d\n", str.c_str(), m_index);

        std::string header = getHeader(str);

        for(int i = 0; i < m_index; i++)
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
