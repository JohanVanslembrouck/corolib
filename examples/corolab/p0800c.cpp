/** 
 *  Filename: p0800c.cpp
 *  Description:
 *        This example illustrates the use of coroutines
 *        in combination with Boost ASIO to implement an echo CommClient.
 *
 *        This example uses 1 CommClient.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *
 */
 
#include <thread>
#include <iostream>
#include <string>
#include <stdarg.h>
#include <stdio.h>

// -----------------------------------------------------------------

/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */
 
const int PRI1 = 0x01;
const int PRI2 = 0x02;
const int PRI3 = 0x04;
const int PRI4 = 0x08;

uint64_t threadids[128];

int get_thread_number64(uint64_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

int get_thread_number32(uint32_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

uint64_t get_thread_id()
{
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t) (*ptr);
}

const int priority = 0x0F;

void print(int pri, const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    if (priority & pri)
        fprintf(stderr, "%02d: %s", threadid, msg);
}

// -----------------------------------------------------------------
// Auxiliary awaitable that allows a coroutine to return control to main().
// Does not define await_ready, await_suspend and await_resume (not needed)
// because main() will (can) not co_await an async_task object.

#include <experimental/resumable>

template<typename T>
struct async_task {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    async_task(const async_task& s) = delete;

    async_task(async_task&& s)
        : coro(s.coro) {
        print(PRI2, "%p: async_task::async_task(async_task&& s)\n", this);
        s.coro = nullptr;
    }

    ~async_task() {
        print(PRI2, "%p: async_task::~async_task()\n", this);
    }

    async_task(handle_type h)
        : coro(h) {
        print(PRI2, "%p: async_task::async_task(handle_type h)\n", this);
    }

    async_task& operator = (const async_task&) = delete;

    async_task& operator = (async_task&& s) {
        print(PRI2, "%p: async_task::async_task = (async_task&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    struct promise_type {

        friend struct async_task;

        promise_type() :
            m_ready(false)
        {
            print(PRI2, "%p: async_task::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: async_task::promise_type::~promise_type()\n", this);
        }

        auto return_value(T v) {
            print(PRI2, "%p: async_task::promise_type::return_value(T v): begin\n", this);
            value = v;
            m_ready = true;
            print(PRI2, "%p: async_task::promise_type::return_value(T v): end\n", this);
        }

        auto get_return_object() {
            print(PRI2, "%p: async_task::promise_type::get_return_object()\n", this);
            return async_task<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print(PRI2, "%p: async_task::promise_type::initial_suspend()\n", this);
            return std::experimental::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "%p: async_task::promise_type::final_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "%p: async_task::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T value;
        bool m_ready;
    };

    handle_type coro;
};

// -----------------------------------------------------------------
// Core library comprising the base class 'CommService' and
// awaitable type async_operation.

class async_operation;

class CommService {
    friend class async_operation;

public:
    virtual void get_result() {}

protected:
    async_operation* m_async_operation;
};

class async_operation {
public:
    async_operation(CommService* s = nullptr) 
        : m_service(s)
        , m_ready(false)
    {
        print(PRI2, "%p: async_operation::async_operation(CommService* s = %p)\n", this, s);
        m_service->m_async_operation = this;
    }

    ~async_operation() {
        print(PRI2, "%p: async_operation::~async_operation()\n", this);
    }

    void completed() {
        if (m_waiting_coroutine) {
            print(PRI2, "%p: async_operation::completed(): before m_awaiting.resume();\n", this);
            m_awaiting.resume();
            m_ready = true;
            print(PRI2, "%p: async_operation::completed(): after m_awaiting.resume();\n", this);
        }
        else {
            print(PRI2, "%p: async_operation::completed(): m_awaiting not yet initialized!!!\n", this);
        }
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(async_operation& async_) : 
                m_async(async_) 
            {}

            bool await_ready() {
                print(PRI2, "%p: async_operation::await_ready(): return %d;\n", this, m_async.m_ready);
                return m_async.m_ready;
            }

            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
                print(PRI2, "%p: async_operation::await_suspend(...)\n", this);
                m_async.m_awaiting = awaiting;
                m_async.m_waiting_coroutine = true;
            }

            CommService* await_resume() {
                print(PRI2, "%p: async_operation::await_resume()\n", this);
                return m_async.m_service;
            }

        private:
            async_operation& m_async;
        };

        return awaiter{ *this };
    }

private:
    CommService* m_service;
    std::experimental::coroutine_handle<> m_awaiting;
    bool m_waiting_coroutine;
    bool m_ready;
};

// -----------------------------------------------------------------
// Library writers write their asynchronous operations class(es)
// in terms of base class CommService and awaitable type async_operation.

// The following code is based on 
// ..\boost_1_70_0\libs\asio\example\cpp11\timeouts\async_tcp_client.cpp

// async_tcp_client.cpp
// ~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2019 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

const boost::asio::ip::tcp::endpoint ep1{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

class CommClient : public CommService
{
public:
    CommClient(boost::asio::io_context& io_context, 
           boost::asio::ip::tcp::endpoint ep)
        : socket_(io_context),
        deadline_(io_context),
        heartbeat_timer_(io_context),
        client_timer_(io_context),
        m_ep(ep)
    {
        print(PRI2, "%p: CommClient::CommClient()\n", this);
    }

    void start()
    {
        print(PRI2, "%p: CommClient::start()\n", this);
        start_connect();

        deadline_.async_wait(std::bind(&CommClient::check_deadline, this));
    }

    void stop()
    {
        print(PRI2, "%p: CommClient::stop()\n", this);
        stopped_ = true;
        boost::system::error_code ignored_error;
        socket_.close(ignored_error);
        deadline_.cancel();
        heartbeat_timer_.cancel();
    }

    async_operation start_connecting()
    {
        print(PRI2, "%p: CommClient::start_connecting()\n", this);
        start_connect();
        return async_operation{ this };
    }

    async_operation start_writing(const char* str, int size)
    {
        print(PRI2, "%p: CommClient::start_writing()\n", this);
        start_write(str, size);
        return async_operation{ this };
    }

    async_operation start_reading(const char ch = '\n')
    {
        print(PRI2, "%p: CommClient::start_reading()\n", this);
        start_read(ch);
        return async_operation{ this };
    }

    async_operation start_timer(int ms)
    {
        print(PRI1, "%p: CommClient::start_timer(%d)\n", this, ms);
        start_tmr(ms);
        return async_operation{ this };
    }

    void get_result() override
    {
        print(PRI2, "%p: CommClient::get_result()\n", this);

        // Extract the newline-delimited message from the buffer.
        std::string line(input_buffer_.substr(0, n_ - 1));
        input_buffer_.erase(0, n_);

        // Empty messages are heartbeats and so ignored.
        if (!line.empty())
        {
            print(PRI2, "%p: Received: %s\n", this, line.c_str());
        }
    }

private:
    void start_connect()
    {
        print(PRI2, "%p: CommClient::start_connect(): m_async_operation = %p\n", this, m_async_operation);

        stopped_ = false;

        std::vector<boost::asio::ip::tcp::endpoint> eps;
        eps.push_back(m_ep);

        // Set a deadline for the connect operation.
        deadline_.expires_after(std::chrono::seconds(5));

        // Start the asynchronous connect operation.
        boost::asio::async_connect(
            socket_, 
            eps,
            [&](const boost::system::error_code& error,
                const tcp::endpoint& /*result_endpoint*/)
            {
                print(PRI2, "%p: CommClient::handle_connect(): entry\n", this);

                if (stopped_)
                    return;

                if (!socket_.is_open())
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect timed out\n", this);
                    start_connect();
                }

                else if (error)
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect error: %d\n", this, error);
                    socket_.close();
                    start_connect();
                }
                else
                {
                    print(PRI2, "%p: CommClient::handle_connect(): successful connection: m_async_operation = %p\n", this, m_async_operation);
                    if (m_async_operation)
                    {
                        m_async_operation->completed();
                    }
                }
                print(PRI2, "%p: CommClient::handle_connect(): exit\n", this);
            });
    }

    void start_write(const char* str, int size)
    {
        static int counter = 0;

        print(PRI2, "%p: CommClient::start_write(): m_async_operation = %p\n", this, m_async_operation);

        if (stopped_)
            return;

        boost::asio::async_write(
            socket_, 
            boost::asio::buffer(str, size),
            [&](const boost::system::error_code& error,
                std::size_t /*result_n*/)
            {
                print(PRI2, "%p: CommClient::handle_write(): entry\n", this);

                if (stopped_)
                    return;

                if (!error)
                {
                    print(PRI2, "%p: CommClient::handle_write(): successful write: m_async_operation = %p\n", this, m_async_operation);
                    if (m_async_operation)
                    {
                        m_async_operation->completed();
                    }
                }
                else
                {
                    print(PRI2, "%p: Error on write: %s\n", this, error.message().c_str());
                    stop();
                }
                print(PRI2, "%p: CommClient::handle_write(): exit\n", this);
            });
    }

    void start_read(const char ch = '\n')
    {
        print(PRI2, "%p: CommClient::start_read(): m_async_operation = %p\n", this, m_async_operation);

        // Set a deadline for the read operation.
        deadline_.expires_after(std::chrono::seconds(10));

        boost::asio::async_read_until(
            socket_,
            boost::asio::dynamic_buffer(input_buffer_), ch,
            [&](const boost::system::error_code& error,
                std::size_t n)
            {
                print(PRI2, "%p: CommClient::handle_read(): entry\n", this);

                if (stopped_)
                    return;

                if (!error)
                {
                    n_ = n;
#if 0
                    // Extract the newline-delimited message from the buffer.
                    std::string line(input_buffer_.substr(0, n - 1));
                    input_buffer_.erase(0, n);

                    if (!line.empty())
                    {
                        print(PRI2, "%p: Received: %s\n", this, line.c_str());
                    }
#endif
                    print(PRI2, "%p: CommClient::handle_read(): successful read: m_async_operation = %p\n", this, m_async_operation);
                    if (m_async_operation)
                    {
                        m_async_operation->completed();
                    }
                }
                else
                {
                    print(PRI2, "%p: Error on receive: %s\n", this, error.message().c_str());
                    stop();
                }
                print(PRI2, "%p: CommClient::handle_read(): exit\n", this);
            });
    }

    void start_tmr(int ms)
    {
        print(PRI2, "%p: CommClient::start_tmr(): m_async_operation = %p\n", this, m_async_operation);

        client_timer_.expires_after(std::chrono::milliseconds(ms));
        client_timer_.async_wait(
            [&](const boost::system::error_code& error)
            {
                print(PRI1, "%p: CommClient::handle_timer()\n", this);

                if (stopped_)
                    return;

                if (!error)
                {
                    print(PRI2, "%p: CommClient::handle_timer(): m_async_operation = %p\n", this, m_async_operation);
                    if (m_async_operation)
                    {
                        m_async_operation->completed();
                    }
                }
            });
    }

    void check_deadline()
    {
        print(PRI2, "%p: CommClient::check_deadline()\n", this);

        if (stopped_)
            return;

        if (deadline_.expiry() <= steady_timer::clock_type::now())
        {
            socket_.close();
            deadline_.expires_at(steady_timer::time_point::max());
        }

        // Put the actor back to sleep.
        deadline_.async_wait(std::bind(&CommClient::check_deadline, this));
    }

private:
    bool stopped_ = false;
    tcp::socket socket_;
    std::string input_buffer_;
    std::size_t n_;
    steady_timer deadline_;
    steady_timer heartbeat_timer_;
    steady_timer client_timer_;
    boost::asio::ip::tcp::endpoint m_ep;
};

// -----------------------------------------------------------------
// Application level
// -----------------------------------------------------------------

async_task<int> mainflow(CommClient& c)
{
    print(PRI2, "mainflow: begin\n");

    for (int i = 0; i < 30; i++)
    {
        print(PRI2, "mainflow: async_operation sc = start_connecting();\n");
        async_operation sc = c.start_connecting();
        print(PRI2, "mainflow: co_await sc;\n");
        co_await sc;

        if (i == 0)
        {
            print(PRI2, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(3000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }

        print(PRI2, "mainflow: async_operation sw = Cc.start_writing(...);\n");
        async_operation sw = c.start_writing("This is string 1 to echo\n", strlen("This is string 1 to echo\n") + 1);
        print(PRI2, "mainflow: co_await sw;\n");
        co_await sw;

        print(PRI2, "mainflow: async_operation sr = c.start_reading();\n");
        async_operation sr = c.start_reading();

        print(PRI2, "mainflow: co_await sr;\n");
        CommService* ssr = co_await sr;
        ssr->get_result();

        //print(PRI2, "mainflow: std::this_thread::sleep_for(std::chrono::milliseconds(100));\n");
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        print(PRI1, "mainflow: async_operation st = c.start_timer(100);\n");
        async_operation st = c.start_timer(100);

        print(PRI1, "mainflow: co_await st;\n");
        co_await st;

        print(PRI2, "mainflow: stop();\n");
        c.stop();
    }

    print(PRI2, "mainflow: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

int main()
{
    boost::asio::io_context ioContext;

    print(PRI2, "main: CommClient c1(ioContext);\n");
    CommClient c1(ioContext, ep1);

    print(PRI2, "main: async_task<int> si = mainflow(c1);\n");
    async_task<int> si = mainflow(c1);

    print(PRI2, "main: ioContext.run();\n");
    ioContext.run();

    print(PRI2, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI2, "main: return 0;\n");
    return 0;
}
