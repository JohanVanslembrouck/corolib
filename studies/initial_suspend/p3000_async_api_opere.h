/**
 * @file p3000_async_api_opere.h
 * @brief
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _P3000_ASYNC_API_OPERE_H_
#define _P3000_ASYNC_API_OPERE_H_

#include <coroutine>

#include "p3000_async_api.h"

template<typename TYPE>
class async_oper : public async_oper_baseT<TYPE> {
public:
    async_oper(int index) {
        print(PRI2, "async_oper::async_oper(index = %d)\n", index);
        async_oper_bases[index] = this;
        m_index = index;
    }
    ~async_oper() {
        print(PRI2, "async_oper::~async_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI2, "async_oper::await_ready() => %d\n", async_oper_baseT<TYPE>::m_ready);
        return async_oper_baseT<TYPE>::m_ready;
    }
    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI2, "async_oper::await_suspend(...)\n");
        async_oper_baseT<TYPE>::m_awaiting = awaiting;
    }
    TYPE await_resume() {
        print(PRI2, "async_oper::await_resume()\n");
        return async_oper_baseT<TYPE>::m_result;
    }

private:
    int m_index = -1;
};

template<>
class async_oper<void> : public async_oper_baseT<void> {
public:
    async_oper(int index) {
        print(PRI2, "async_oper::async_oper(index = %d)\n", index);
        async_oper_bases[index] = this;
        m_index = index;
    }
    ~async_oper() {
        print(PRI2, "async_oper::~async_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI2, "async_oper::await_ready() => %d\n", async_oper_baseT<void>::m_ready);
        return async_oper_baseT<void>::m_ready;
    }
    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI2, "async_oper::await_suspend(...)\n");
        async_oper_baseT<void>::m_awaiting = awaiting;
    }
    void await_resume() {
        print(PRI2, "async_oper::await_resume()\n");
    }

private:
    int m_index = -1;
};

async_oper<bool> start_create() {
    int index = get_free_index();
    print(PRI2, "start_create(): index = %d\n", index);
    async_oper<bool> ret{ index };
    async_create(
        [index](bool val) {
            completionHandler(index, val);
        });
    return ret;
}

async_oper<int> start_open() {
    int index = get_free_index();
    print(PRI2, "start_open(): index = %d\n", index);
    async_oper<int> ret{ index };
    async_open(
        [index](int val) {
            completionHandler(index, val);
        });
    return ret;
}

async_oper<int> start_write(char* buffer) {
    int index = get_free_index();
    print(PRI2, "start_write(): index = %d\n", index);
    async_oper<int> ret{ index };
    async_write(buffer,
        [index](int val) {    // Could run on a dedicated thread
            completionHandler(index, val);
        });
    // In case of threads:
    // The completion handler may have run here and resumed the operation 
    // and its calling coroutine
    return ret;
}

async_oper<std::string> start_read() {
    int index = get_free_index();
    print(PRI2, "start_read(): index = %d\n", index);
    async_oper<std::string> ret{ index };
    async_read(
        [index](std::string val) {
            completionHandler(index, val);
        });
    return ret;
}

async_oper<bool> start_close() {
    int index = get_free_index();
    print(PRI2, "start_close(): index = %d\n", index);
    async_oper<bool> ret{ index };
    async_close(
        [index](bool val) {
            completionHandler(index, val);
        });
    return ret;
}

async_oper<bool> start_remove() {
    int index = get_free_index();
    print(PRI2, "start_remove(): index = %d\n", index);
    async_oper<bool> ret{ index };
    async_remove(
        [index](bool val) {
            completionHandler(index, val);
        });
    return ret;
}

#endif