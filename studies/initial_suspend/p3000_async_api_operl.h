/**
 * @file p3000_async_api_operl.h
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P3000_ASYNC_API_OPERL_H_
#define _P3000_ASYNC_API_OPERL_H_

#include <coroutine>

#include "p3000_async_api.h"
#include "tracker1.h"

class async_create_oper : public async_oper_base {
public:
    async_create_oper() {
        m_index = get_free_index();
        print(PRI1, "async_create_oper::async_create_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_create_oper() {
        print(PRI1, "async_create_oper::~async_create_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_create_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_create_oper::await_suspend(...)\n");
        async_create(
            [this, handle]() {
                completionHandler(m_index);
                print(PRI1, "async_create_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
    }
    int await_resume() {
        print(PRI1, "async_create_oper::await_resume()\n");
        return m_result;
    }
private:
    int m_index = -1;
};

class async_open_oper : public async_oper_base {
public:
    async_open_oper() {
        m_index = get_free_index();
        print(PRI1, "async_open_oper::async_open_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_open_oper() {
        print(PRI1, "async_open_oper::~async_open_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_open_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_open_oper::await_suspend(...)\n");
        async_open(
            [this, handle]() {
                completionHandler(m_index);
                print(PRI1, "async_open_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
    }
    int await_resume() {
        print(PRI1, "async_open_oper::await_resume()\n");
        return m_result;
    }
private:
    int m_index = -1;
};

class async_write_oper : public async_oper_base {
public:
    async_write_oper() {
        m_index = get_free_index();
        print(PRI1, "async_write_oper::async_write_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_write_oper() {
        print(PRI1, "async_write_oper::~async_write_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_write_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_write_oper::await_suspend(...)\n");
        async_write(m_buffer,
            [this, handle]() {    // Could run on a dedicated thread
                completionHandler(m_index);
                print(PRI1, "async_write_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
        // In case of threads:
        // The ccmpletion handler may have run here and resumed the coroutine
    }
    int await_resume() {
        print(PRI1, "async_write_oper::await_resume()\n");
        return m_result;
    }
private:
    char m_buffer[100];
    int m_index = -1;
};

class async_read_oper : public async_oper_base {
public:
    async_read_oper() {
        m_index = get_free_index();
        print(PRI1, "async_read_oper::async_read_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_read_oper() {
        print(PRI1, "async_read_oper::~async_read_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_read_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_read_oper::await_suspend(...)\n");
        async_read(
            [this, handle]() {
                completionHandler(m_index);
                print(PRI1, "async_read_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
    }
    int await_resume() {
        print(PRI1, "async_read_oper::await_resume()\n");
        return m_result;
    }
private:
    int m_index = -1;
};

class async_close_oper : public async_oper_base {
public:
    async_close_oper() {
        m_index = get_free_index();
        print(PRI1, "async_close_oper::async_close_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_close_oper() {
        print(PRI1, "async_close_oper::~async_close_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_close_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_close_oper::await_suspend(...)\n");
        async_close(
            [this, handle]() {
                completionHandler(m_index);
                print(PRI1, "async_close_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
    }
    int await_resume() {
        print(PRI1, "async_close_oper::await_resume()\n");
        return m_result;
    }
private:
    int m_index = -1;
};

class async_remove_oper : public async_oper_base {
public:
    async_remove_oper() {
        m_index = get_free_index();
        print(PRI1, "async_remove_oper::async_remove_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = this;
    }
    ~async_remove_oper() {
        print(PRI1, "async_remove_oper::~async_remove_oper(): m_index = %d\n", m_index);
        async_oper_bases[m_index] = nullptr;
    }

    bool await_ready() {
        print(PRI1, "async_remove_oper::await_ready() => %d\n", false);
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) {
        print(PRI1, "async_remove_oper::await_suspend(...)\n");
        async_remove(
            [this, handle]() {
                completionHandler(m_index);
                print(PRI1, "async_read_oper::await_suspend(...): handle.resume();\n");
                tracker1_obj.nr_resumptions++;
                handle.resume();
            });
    }
    int await_resume() {
        print(PRI1, "async_remove_oper::await_resume()\n");
        return m_result;
    }
private:
    int m_index = -1;
};

#endif
