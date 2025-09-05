/**
 * @file async_file.h
 * @brief class async_file simulates file access using asynchronous versions
 * of typical file functions (create, open, read, write, close, remove).
 * There is no access to a real file. Only strings can be written or read.
 * The information written is added to a member variable m_content.
 * The read operation returns the complete content of this variable.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _ASYNC_FILE_H_
#define _ASYNC_FILE_H_

#include <corolib/commservice.h>
#include <corolib/print.h>

#include "eventqueue.h"

using FunctionVoidVoid = std::function<void(void)>;

using namespace corolib;

class async_file : public CommService
{
public:
    async_file(EventQueueFunctionVoidVoid& eventQueue)
        : m_eventQueue(eventQueue)
    {

    }

    async_operation<int> start_create(std::string filename)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_create([this, index]() {
            print(PRI5, "async_create handler: begin\n");
            m_opened = true;
            completionHandler_v(index);
            print(PRI5, "async_create handler: end\n");
        });
        return ret;
    }

    async_operation<int> start_open(std::string filename)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_open([this, index]() {
            print(PRI5, "async_open handler: begin\n");
            m_opened = true;
            completionHandler_v(index);
            print(PRI5, "async_open handler: end\n");
        });
        return ret;
    }

    async_operation<int> start_write(std::string str)
    {
        print(PRI1, "start_write(%s)\n", str.c_str());
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        if (this->m_content.length() >= 100)
            // Avoids buffer/stack overflow
            this->m_content = str;
        else
            this->m_content += str;
        this->m_content += "\n";
        if (!m_opened)
            ret.set_result_and_complete(-1);
        else
            async_write([this, index]() {
                print(PRI5, "async_write handler: begin\n");
                completionHandler_v(index);
                print(PRI5, "async_write handler: end\n");
            });
        return ret;
    }

    struct read_result
    {
        int result;
        std::string content;
    };

    async_operation<read_result> start_read()
    {
        int index = get_free_index();
        async_operation<read_result> ret{ this, index };
        if (!m_opened)
            ret.set_result_and_complete({-1, ""});
        else
            async_read([this, index]() {
                print(PRI5, "async_read handler: begin\n");
                completionHandler<read_result>(index, { 0, this->m_content });
                print(PRI5, "async_read handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_close()
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_close([this, index]() {
            print(PRI5, "async_close handler: begin\n");
            m_opened = false;
            completionHandler_v(index);
            print(PRI5, "async_close handler: end\n");
        });
        return ret;
    }

    async_operation<int> start_remove(std::string filename)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_remove([this, index]() {
            print(PRI5, "async_remove handler: begin\n");
            completionHandler_v(index);
            print(PRI5, "async_remove handler: end\n");
        });
        return ret;
    }

protected:
    void async_create(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_create()\n");
        m_eventQueue.push(std::move(f));
    }

    void async_open(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_open()\n");
        m_eventQueue.push(std::move(f));
    }

    void async_write(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_write()\n");
        m_eventQueue.push(std::move(f));
    }

    void async_read(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_read()\n");
        m_eventQueue.push(std::move(f));
    }

    void async_close(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_close()\n");
        m_eventQueue.push(std::move(f));
    }

    void async_remove(FunctionVoidVoid&& f)
    {
        print(PRI5, "async_remove()\n");
        m_eventQueue.push(std::move(f));
    }

private:
    EventQueueFunctionVoidVoid& m_eventQueue;
    bool m_opened = false;
    std::string m_content{};
};

#endif
