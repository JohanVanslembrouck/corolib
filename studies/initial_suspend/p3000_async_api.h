/**
 * @file p3000_async_api.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P3000_ASYNC_API_H_
#define _P3000_ASYNC_API_H_

#include <functional>

#include "eventqueuethr.h"
#include "print.h"

extern EventQueueThrFunctionVoidVoid evqueuethr;

using FunctionVoidVoid = std::function<void(void)>;

void async_create(FunctionVoidVoid f)        { print(PRI1, "async_create()\n"); evqueuethr.push(f); }
void async_open(FunctionVoidVoid f)          { print(PRI1, "async_open()\n");   evqueuethr.push(f); }
void async_write(char* , FunctionVoidVoid f) { print(PRI1, "async_write()\n");  evqueuethr.push(f); }
void async_write(FunctionVoidVoid f)         { print(PRI1, "async_write()\n");  evqueuethr.push(f); }
void async_read(FunctionVoidVoid f)          { print(PRI1, "async_read()\n");   evqueuethr.push(f); }
void async_close(FunctionVoidVoid f)         { print(PRI1, "async_close()\n");  evqueuethr.push(f); }
void async_remove(FunctionVoidVoid f)        { print(PRI1, "async_remove()\n"); evqueuethr.push(f); }

#include <coroutine>

class async_oper_base {
public:
    void set_result_and_resume(int result = 0) {
        print(PRI1, "async_oper::set_result_and_resume()\n");
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                m_awaiting.resume();
            }
        }
    }
protected:
    std::coroutine_handle<> m_awaiting = nullptr;
    bool m_ready = false;
    int m_result = -1;
};

extern int start_index;

int get_free_index() {
    return start_index++;
}

extern async_oper_base* async_oper_bases[32];

void completionHandler(int index) {
    print(PRI1, "completionHandler(%d)\n", index);
    if (async_oper_bases[index])
        async_oper_bases[index]->set_result_and_resume();
}

#endif
