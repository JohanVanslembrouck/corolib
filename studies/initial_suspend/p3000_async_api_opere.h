/**
 * @file p3000_async_api_opere.h
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P3000_ASYNC_API_OPERE_H_
#define _P3000_ASYNC_API_OPERE_H_

#include <coroutine>

#include "p3000_async_api.h"

class async_oper : public async_oper_base {
public:
    async_oper(int index) {
        print(PRI1, "async_oper::async_oper(index = %d)\n", index);
        async_oper_bases[index] = this;
    }
    bool await_ready() {
        print(PRI1, "async_oper::await_ready() => %d\n", m_ready);
        return m_ready;
    }
    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI1, "async_oper::await_suspend(...)\n");
        m_awaiting = awaiting;
    }
    int await_resume() {
        print(PRI1, "async_oper::await_resume()\n");
        return m_result;
    }
};

async_oper start_create() {
    int index = get_free_index();
    async_oper ret{ index };
    async_create(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

async_oper start_open() {
    int index = get_free_index();
    async_oper ret{ index };
    async_open(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

async_oper start_write() {
    int index = get_free_index();
    async_oper ret{ index };
    async_write(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

async_oper start_write(char* buffer) {
    int index = get_free_index();
    async_oper ret{ index };
    async_write(buffer,
        [index]() {    // Could run on a dedicated thread
            completionHandler(index);
        });
    // In case of threads:
    // The completion handler may have run here and resumed the operation 
    // and its calling coroutine
    return ret;
}

async_oper start_read() {
    int index = get_free_index();
    async_oper ret{ index };
    async_read(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

async_oper start_close() {
    int index = get_free_index();
    async_oper ret{ index };
    async_close(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

async_oper start_remove() {
    int index = get_free_index();
    async_oper ret{ index };
    async_remove(
        [index]() {
            completionHandler(index);
        });
    return ret;
}

#endif