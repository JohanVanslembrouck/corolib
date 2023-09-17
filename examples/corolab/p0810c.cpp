/** 
 *  Filename: p0810c.cpp
 *  Description:
 *        This example illustrates the use of coroutines
 *        in combination with Boost ASIO to implement an echo CommClient.
 *
 *        This example uses 3 clients.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#include <boost/asio.hpp>

#include <thread>
#include <iostream>
#include <string>

const boost::asio::ip::tcp::endpoint ep{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

#include <coroutine>

#include "print.h"
#include "tracker.h"
#include "csemaphore.h"

// -----------------------------------------------------------------

template<typename T>
struct async_task : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    async_task(const async_task& s) = delete;

    async_task(async_task&& s)
        : coro(s.coro) {
        print(PRI2, "%p: async_task::async_task(async_task&& s)\n", this);
        s.coro = nullptr;
    }

    ~async_task() {
        print(PRI2, "%p: async_task::~async_task()\n", this);
        if (coro && coro.done())
            coro.destroy();
    }

    async_task(handle_type h)
        : coro(h) {
        print(PRI2, "%p: async_task::async_task(handle_type h)\n", this);
    }

    async_task& operator = (const async_task&) = delete;

    async_task& operator = (async_task&& s) {
        print(PRI2, "%p: async_task::operator = (async_task&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print(PRI2, "%p: async_task::get()\n", this);
        if (!coro.promise().m_ready) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().sema.wait();
        }
        return coro.promise().value;
    }

    struct promise_type : private promise_type_tracker {

        friend struct async_task;

        promise_type() :
            m_value{},
            m_awaiting(nullptr),
            m_ready(false),
            m_wait_for_signal(false),
            m_waiting_coroutine(false) {
            print(PRI2, "%p: async_task::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: async_task::promise_type::~promise_type()\n", this);
        }

        auto return_value(T v) {
            print(PRI2, "%p: async_task::promise_type::return_value(T v): begin\n", this);
            m_value = v;
            m_ready = true;
            if (m_waiting_coroutine) {
                print(PRI2, "%p: async_task::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print(PRI2, "%p: async_task::promise_type::return_value(T v): after m_awaiting.resume();\n", this);
            }
            if (m_wait_for_signal) {
                print(PRI2, "%p: async_task::promise_type::return_value(T v): before sema.signal();\n", this);
                sema.signal();
                print(PRI2, "%p: async_task::promise_type::return_value(T v): after sema.signal();\n", this);
            }
            print(PRI2, "%p: async_task::promise_type::return_value(T v): end\n", this);
        }

        auto get_return_object() {
            print(PRI2, "%p: async_task::promise_type::get_return_object()\n", this);
            return async_task<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print(PRI2, "%p: async_task::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "%p: async_task::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "%p: async_task::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        CSemaphore sema;
        std::coroutine_handle<> m_awaiting;
        bool m_ready;
        bool m_wait_for_signal;
        bool m_waiting_coroutine;
    };

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:
            awaiter(async_task& async_task_)
                : m_async_task(async_task_)
            {}

            bool await_ready()
            {
                const bool ready = m_async_task.coro.done();
                print(PRI2, "%p: m_async_task::await_ready(): return %d;\n", this, ready);
                return ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting)
            {
                print(PRI2, "%p: m_async_task::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_async_task.coro.promise().m_awaiting = awaiting;
            }

            T await_resume()
            {
                print(PRI2, "%p: m_async_task::await_resume()\n", this);
                const T r = m_async_task.coro.promise().m_value;
                return r;
            }

        private:
            async_task& m_async_task;
        };

        return awaiter{ *this };
    }

    handle_type coro;
};

// -----------------------------------------------------------------

struct oneway_task
{
    oneway_task() {
        print(PRI2, "%p: oneway_task::oneway_task()\n", this);
    }

    ~oneway_task() {
        print(PRI2, "%p: oneway_task::~oneway_task()\n", this);
    }

    struct promise_type
    {
        promise_type()
        {
            print(PRI2, "%p: oneway_task::promise_type::promise_type()\n", this);
        }

        ~promise_type()
        {
            print(PRI2, "%p: oneway_task::promise_type::~promise_type()\n", this);
        }

        std::suspend_never initial_suspend() {
            print(PRI2, "%p: oneway_task::promise_type::initial_suspend()\n", this);
            return {};
        }

        std::suspend_never final_suspend() noexcept {
            print(PRI2, "%p: oneway_task::promise_type::final_suspend()\n", this);
            return {};
        }

        void unhandled_exception() {
            print(PRI2, "%p: oneway_task::promise_type::unhandled_exception()\n", this);
            std::terminate();
        }

        oneway_task get_return_object() {
            print(PRI2, "%p: oneway_task::promise_type::get_return_object()\n", this);
            return {};
        }

        void return_void() {
            print(PRI2, "%p: oneway_task::promise_type::return_void()\n", this);
        }
    };
};

// -----------------------------------------------------------------

struct auto_reset_event {

    std::coroutine_handle<> m_awaiting;
    bool m_ready;

    auto_reset_event()
        : m_awaiting(nullptr)
        , m_ready(false) {
        print(PRI2, "%p: auto_reset_event::auto_reset_event()\n", this);
    }

    auto_reset_event(const auto_reset_event&) = delete;
    auto_reset_event& operator = (const auto_reset_event&) = delete;

    auto_reset_event(auto_reset_event&& s)
        : m_awaiting(s.m_awaiting)
        , m_ready(s.m_ready) {
        print(PRI2, "%p: auto_reset_event::auto_reset_event(auto_reset_event&& s)\n", this);
        s.m_awaiting = nullptr;
        s.m_ready = false;
    }

    auto_reset_event& operator = (auto_reset_event&& s) {
        print(PRI2, "%p: auto_reset_event::auto_reset_event = (auto_reset_event&& s)\n", this);
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        return *this;
    }

    void resume() {
        print(PRI2, "%p: auto_reset_event::resume(): before m_awaiting.resume();\n", this);
        m_ready = true;
        if (m_awaiting && !m_awaiting.done())
            m_awaiting.resume();
        print(PRI2, "%p: auto_reset_event::resume(): after m_awaiting.resume();\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:
            awaiter(auto_reset_event& are_) :
                m_are(are_) {
                print(PRI2, "%p: auto_reset_event::awaiter(auto_reset_event& ars_)\n", this);
            }

            bool await_ready() {
                print(PRI2, "%p: auto_reset_event::await_ready(): return false\n", this);
                return m_are.m_ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print(PRI2, "%p: auto_reset_event::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_are.m_awaiting = awaiting;
            }

            void await_resume() {
                print(PRI2, "%p: void auto_reset_event::await_resume()\n", this);
                m_are.m_ready = false;
            }

        private:
            auto_reset_event& m_are;
        };

        return awaiter{ *this };
    }
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
    static const int NROPERATIONS = 4;
    CommService()
        : index(-1)
    {
        for (int i = 0; i < NROPERATIONS; i++)
            m_async_operations[i] = nullptr;
    }

    int index = 0;
    async_operation* m_async_operations[NROPERATIONS];
};

//--------------------------------------------------------------

template<typename T>
struct eager {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    eager(const eager& s) = delete;

    eager(eager&& s)
        : coro(s.coro) {
        print(PRI2, "%p: eager::eager(eager&& s)\n", this);
        s.coro = nullptr;
    }

    ~eager() {
        print(PRI2, "%p: eager::~eager()\n", this);
        if (coro)
            if (coro.done())
                coro.destroy();
    }

    eager() {
        print(PRI2, "%p: eager::eager()\n", this);
    }

    eager(handle_type h)
        : coro(h) {
        print(PRI2, "%p: eager::eager(handle_type h)\n", this);
    }

    eager& operator = (const eager&) = delete;

    eager& operator = (eager&& s) {
        print(PRI2, "%p: eager::eager = (eager&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print(PRI2, "%p: eager::get(); coro.done() = %d\n", this, coro.done());
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(eager& eager_) :
                m_eager(eager_)
            {}

            bool await_ready() {
                const bool ready = m_eager.coro.done();
                print(PRI2, "%p: eager::await_ready(): return %d;\n", this, ready);
                return ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print(PRI2, "%p: eager::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_eager.coro.promise().m_awaiting = awaiting;
            }

            T await_resume() {
                print(PRI2, "%p: eager::await_resume()\n", this);
                const T r = m_eager.coro.promise().m_value;
                return r;
            }

        private:
            eager& m_eager;
        };

        return awaiter{ *this };
    }

    struct promise_type {

        friend struct eager;

        promise_type() :
            m_value{},
            m_awaiting(nullptr),
            m_wait_for_signal(false) {
            print(PRI2, "%p: eager::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: eager::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print(PRI2, "%p: eager::promise_type::return_value(T v): begin\n", this);
            m_value = v;
            if (m_awaiting) {
                print(PRI2, "%p: eager::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print(PRI2, "%p: eager::promise_type::return_value(T v): after m_awaiting.resume();\n", this);
            }
            if (m_wait_for_signal) {
                print(PRI2, "%p: eager::promise_type::return_value(T v): before m_sema.signal();\n", this);
                m_sema.signal();
                print(PRI2, "%p: eager::promise_type::return_value(T v): after m_sema.signal();\n", this);
            }
            print(PRI2, "%p: eager::promise_type::return_value(T v): end\n", this);
        }

        auto get_return_object() {
            print(PRI2, "%p: eager::promise_type::get_return_object()\n", this);
            return eager<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print(PRI2, "%p: eager::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "%p: eager::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "%p: eager::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        std::coroutine_handle<> m_awaiting;
        CSemaphore m_sema;
        bool m_wait_for_signal;
    };

    handle_type coro;
};

// -----------------------------------------------------------------

class async_operation {
public:
    async_operation(CommService* s = nullptr, int index = 0)
        : m_service(s)
        , m_awaiting(nullptr)
        , m_index(index)
        , m_ready(false)
    {
        print(PRI2, "%p: async_operation::async_operation(CommService* s = %p)\n", this, s);
        if (m_service) {
            if (!m_service->m_async_operations[index]) {
                m_service->m_async_operations[index] = this;
            }
            else {
                m_ready = true;
                m_service->m_async_operations[index] = nullptr;
            }
        }
    }

    ~async_operation() {
        print(PRI2, "%p: async_operation::~async_operation()\n", this);
    }

    async_operation(const async_operation& s) = delete;

    async_operation(async_operation&& s)
        : m_service(s.m_service)
        , m_awaiting(s.m_awaiting)
        , m_index(s.m_index)
        , m_ready(s.m_ready)
    {
        print(PRI2, "%p: async_operation::async_operation(async_operation&& s = %p)\n", this, &s);

        print(PRI2, "%p: async_operation::async_operation(async_operation&& s): m_service = %p\n", this, m_service);
        print(PRI2, "%p: async_operation::async_operation(async_operation&& s): m_awaiting = %p\n", this, m_awaiting);
        print(PRI2, "%p: async_operation::async_operation(async_operation&& s): m_ready = %d\n", this, m_ready);

        // Tell the CommService we are at another address after the move.
        m_service->m_async_operations[m_index] = this;

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        //s.m_index = -1;
        s.m_ready = false;
    }

    async_operation& operator = (const async_operation&) = delete;

    async_operation& operator = (async_operation&& s) {
        print(PRI2, "%p: async_operation::operator = (async_operation&& s = %p)\n", this, &s);

        m_service = s.m_service;
        m_awaiting = s.m_awaiting;
        m_index = s.m_index;
        m_ready = s.m_ready;
        // Tell the CommService we are at ]nother address after the move.
        m_service->m_async_operations[m_index] = this;

        print(PRI2, "%p: async_operation::operator = (async_operation&& s): m_service = %p\n", this, m_service);
        print(PRI2, "%p: async_operation::operator = (async_operation&& s): m_awaiting = %p\n", this, m_awaiting);
        print(PRI2, "%p: async_operation::operator = (async_operation&& s): m_ready = %d\n", this, m_ready);

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        //s.m_index = -1;
        s.m_ready = false;
        return *this;
    }

    void completed() {
        m_ready = true;
        if (m_awaiting) {
            print(PRI2, "%p: async_operation::completed(): before m_awaiting.resume();\n", this);
            m_awaiting.resume();
            print(PRI2, "%p: async_operation::completed(): after m_awaiting.resume();\n", this);
        }
        else {
            print(PRI2, "%p: async_operation::completed(): m_awaiting not yet initialized!\n", this);
            print(PRI2, "%p: async_operation::completed(): operation completed before co_waited!\n", this);
        }
    }

    void get() {
        print(PRI2, "%p: async_operation::get()\n", this);
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
                print(PRI2, "%p: async_operation::await_ready(): m_async = %p, return %d;\n", this, &m_async, m_async.m_ready);
                return m_async.m_ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print(PRI2, "%p: async_operation::await_suspend(...): m_async = %p\n", this, &m_async);
                m_async.m_awaiting = awaiting;
            }

            CommService* await_resume() {
                print(PRI2, "%p: async_operation::await_resume(): m_async = %p\n", this, &m_async);
                return m_async.m_service;
            }

        private:
            async_operation& m_async;
        };

        return awaiter{ *this };
    }

private:
    CommService* m_service;
    std::coroutine_handle<> m_awaiting;
    int m_index;
    bool m_ready;
};

// -----------------------------------------------------------------

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <functional>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;
//using std::placeholders::_1;
//using std::placeholders::_2;

async_operation dummy_async_operation;

class CommClient : public CommService
{
public:
    CommClient(boost::asio::io_context& io_context, 
           boost::asio::ip::tcp::endpoint ep)
        : socket_(io_context),
        deadline_(io_context),
        heartbeat_timer_(io_context),
        m_ep(ep)
    {
        print(PRI2, "%p: CommClient::CommClient()\n", this);
    }

    // Called by the user of the CommClient class to initiate the connection process.
    // The endpoints will have been obtained using a tcp::resolver.
    void start()
    {
        print(PRI2, "%p: CommClient::start()\n", this);
        index = (index + 1) % NROPERATIONS;
        start_connect(index);

        // Start the deadline actor. You will note that we're not setting any
        // particular deadline here. Instead, the connect and input actors will
        // update the deadline prior to each asynchronous operation.
        deadline_.async_wait(std::bind(&CommClient::check_deadline, this));
    }

    // This function terminates all the actors to shut down the connection. It
    // may be called by the user of the CommClient class, or by the class itself in
    // response to graceful termination or an unrecoverable error.
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
        index = (index + 1) % NROPERATIONS;
        start_connect(index);
        return async_operation{ this, index };
    }

    async_operation start_writing(const char* str, int size)
    {
        print(PRI2, "%p: CommClient::start_writing()\n", this);
        index = (index + 1) % NROPERATIONS;
        start_write(index, str, size);
        return async_operation{ this, index };
    }

    async_operation start_reading(const char ch = '\n')
    {
        print(PRI2, "%p: CommClient::start_reading()\n", this);
        index = (index + 1) % NROPERATIONS;
        start_read(index, ch);
        return async_operation{ this, index };
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
    void start_connect(const int idx)
    {
        print(PRI2, "%p: CommClient::start_connect()\n", this);

        stopped_ = false;

        std::vector<boost::asio::ip::tcp::endpoint> eps;
        eps.push_back(m_ep);

        tcp::resolver::results_type::iterator endpoint_iter;

        // Set a deadline for the connect operation.
        deadline_.expires_after(std::chrono::seconds(5));

        // Start the asynchronous connect operation.
        boost::asio::async_connect(
            socket_,
            eps,
            [this, idx](const boost::system::error_code& error,
                const tcp::endpoint& /*result_endpoint*/)
            {
                print(PRI2, "%p: CommClient::handle_connect(): entry\n", this);
                async_operation* om_async_operation = m_async_operations[idx];

                if (stopped_)
                    return;

                // The async_operation() function automatically opens the socket at the start
                // of the asynchronous operation. If the socket is closed at this time then
                // the timeout handler must have run first.
                if (!socket_.is_open())
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect timed out\n", this);

                    // Try the next available endpoint.
                    start_connect(idx);
                }

                // Check if the connect operation failed before the deadline expired.
                else if (error)
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect error: %d\n", this, error);

                    // We need to close the socket used in the previous connection attempt
                    // before starting a new one.
                    socket_.close();

                    // Try the next available endpoint.
                    start_connect(idx);
                }

                // Otherwise we have successfully established a connection.
                else
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connection successfully established\n", this);
                    if (om_async_operation)
                    {
                        om_async_operation->completed();
                    }
                    else
                    {
                        print(PRI2, "%p: CommClient::handle_connect(): om_async_operation = null\n", this);
                        m_async_operations[idx] = &dummy_async_operation;
                    }
                }
                m_async_operations[idx] = nullptr;
                print(PRI2, "%p: CommClient::handle_connect(): exit\n\n", this);
            });
    }

    void start_write(const int idx, const char *str, int size)
    {
        print(PRI2, "%p: CommClient::start_write()\n", this);

        if (stopped_)
            return;

        boost::asio::async_write(
            socket_,
            boost::asio::buffer(str, size),
            [this, idx ](const boost::system::error_code& error,
                std::size_t /*result_n*/)
            {
                print(PRI2, "%p: CommClient::handle_write(): entry\n", this);
                async_operation* om_async_operation = m_async_operations[idx];

                if (stopped_)
                    return;

                if (!error)
                {
                    if (om_async_operation)
                    {
                        om_async_operation->completed();
                    }
                    else
                    {
                        print(PRI2, "%p: CommClient::handle_write(): om_async_operation = null\n", this);
                        m_async_operations[idx] = &dummy_async_operation;
                    }
                }
                else
                {
                    print(PRI2, "%p: Error on write: %s\n", this, error.message().c_str());
                    stop();
                }
                m_async_operations[idx] = nullptr;
                print(PRI2, "%p: CommClient::handle_write(): exit\n\n", this);
            });
    }

    void start_read(const int idx, const char ch = '\n')
    {
        print(PRI2, "%p: CommClient::start_read()\n", this);

        // Set a deadline for the read operation.
        deadline_.expires_after(std::chrono::seconds(10));

        boost::asio::async_read_until(
            socket_,
            boost::asio::dynamic_buffer(input_buffer_), ch,
            [this, idx](const boost::system::error_code& error,
                std::size_t n)
            {
                print(PRI2, "%p: CommClient::handle_read(): entry\n", this);
                async_operation* om_async_operation = m_async_operations[idx];

                if (!error)
                {
                    n_ = n;

                    if (om_async_operation)
                    {
                        om_async_operation->completed();
                    }
                    else
                    {
                        print(PRI2, "%p: CommClient::handle_read(): om_async_operation = null\n", this);
                        m_async_operations[idx] = &dummy_async_operation;
                    }
                }
                else
                {
                    print(PRI2, "%p: Error on receive: %s\n", this, error.message().c_str());
                    stop();
                }
                m_async_operations[idx] = nullptr;
                print(PRI2, "%p: CommClient::handle_read(): exit\n\n", this);
            });
    }

    void check_deadline()
    {
        print(PRI2, "%p: CommClient::check_deadline()\n", this);

        if (stopped_)
            return;

        // Check whether the deadline has passed. We compare the deadline against
        // the current time since a new asynchronous operation may have moved the
        // deadline before this actor had a chance to run.
        if (deadline_.expiry() <= steady_timer::clock_type::now())
        {
            // The deadline has passed. The socket is closed so that any outstanding
            // asynchronous operations are cancelled.
            socket_.close();

            // There is no longer an active deadline. The expiry is set to the
            // maximum time point so that the actor takes no action until a new
            // deadline is set.
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
    boost::asio::ip::tcp::endpoint m_ep;
};

// -----------------------------------------------------------------

async_task<int> mainflow(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI2, "mainflow: begin\n");

    for (int i = 0; i < 3; i++)
    {
        print(PRI2, "mainflow: async_operation sc1 = c1.start_connecting();\n");
        async_operation sc1 = c1.start_connecting();
        print(PRI2, "mainflow: async_operation sc2 = c2.start_connecting();\n");
        async_operation sc2 = c2.start_connecting();
        print(PRI2, "mainflow: async_operation sc3 = c3.start_connecting();\n");
        async_operation sc3 = c3.start_connecting();

        print(PRI2, "mainflow: co_await sc1;\n");
        co_await sc1;
        print(PRI2, "mainflow: co_await sc2;\n");
        co_await sc2;
        print(PRI2, "mainflow: co_await sc3;\n");
        co_await sc3;

        print(PRI2, "mainflow: async_operation sw1 = c1.start_writing(...);\n");
        async_operation sw1 = c1.start_writing("This is string 1 to echo\n", strlen("This is string 1 to echo\n") + 1);
        print(PRI2, "mainflow: async_operation sw2 = c2.start_writing(...);\n");
        async_operation sw2 = c2.start_writing("This is string 2 to echo\n", strlen("This is string 2 to echo\n") + 1);
        print(PRI2, "mainflow: async_operation sw3 = c3.start_writing(...);\n");
        async_operation sw3 = c3.start_writing("This is string 3 to echo\n", strlen("This is string 2 to echo\n") + 1);

#if 0
        print(PRI2, "mainflow: co_await sw1;\n");
        co_await sw1;
        print(PRI2, "mainflow: co_await sw2;\n");
        co_await sw2;
        print(PRI2, "mainflow: co_await sw3;\n");
        co_await sw3;
#else
        print(PRI2, "mainflow: co_await sw2;\n");
        co_await sw2;
        print(PRI2, "mainflow: co_await sw1;\n");
        co_await sw1;
        print(PRI2, "mainflow: co_await sw3;\n");
        co_await sw3;
#endif

        print(PRI2, "mainflow: async_operation sr1 = c1.start_reading();\n");
        async_operation sr1 = c1.start_reading('\n');
        print(PRI2, "mainflow: async_operation sr2 = c2.start_reading();\n");
        async_operation sr2 = c2.start_reading('\n');
        print(PRI2, "mainflow: async_operation sr3 = c3.start_reading();\n");
        async_operation sr3 = c3.start_reading('\n');

        print(PRI2, "mainflow: co_await sr1;\n");
        CommService* ssr1 = co_await sr1;
        ssr1->get_result();
        print(PRI2, "mainflow: co_await sr2;\n");
        CommService* ssr2 = co_await sr2;
        ssr2->get_result();
        print(PRI2, "mainflow: co_await sr3;\n");
        CommService* ssr3 = co_await sr3;
        ssr3->get_result();

        print(PRI2, "mainflow: c1.stop();\n");
        c1.stop();
        print(PRI2, "mainflow: c2.stop();\n");
        c2.stop();
        print(PRI2, "mainflow: c3.stop();\n");
        c3.stop();

        print(PRI2, "mainflow: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print(PRI2, "mainflow: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

oneway_task oneClient(CommClient& c1, auto_reset_event& are)
{
    print(PRI2, "oneClient: begin\n");

    print(PRI2, "oneClient: async_operation sc1 = c1.start_connecting();\n");
    async_operation sc1 = c1.start_connecting();

    print(PRI2, "oneClient: co_await sc1;\n");
    co_await sc1;

    print(PRI2, "oneClient: async_operation sw1 = c1.start_writing(...);\n");
    async_operation sw1 = c1.start_writing("This is string 1 to echo\n", 
                                    strlen("This is string 1 to echo\n") + 1);

    print(PRI2, "oneClient: co_await sw1;\n");
    co_await sw1;

    print(PRI2, "oneClient: async_operation sr1 = c1.start_reading();\n");
    async_operation sr1 = c1.start_reading('\n');

    print(PRI2, "oneClient: co_await sr1;\n");
    CommService* ssr1 = co_await sr1;
    ssr1->get_result();

    print(PRI2, "oneClient: c1.stop();\n");
    c1.stop();

    are.resume();
}

// -----------------------------------------------------------------

async_task<int> mainflow2(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow2: begin\n");
    for (int i = 0; i < 3; i++)
    {
        print(PRI2, "mainflow2: -----------------------------------------\n");
        auto_reset_event are1, are2, are3;
    
        print(PRI1, "mainflow2: oneClient(c1, are1);\n");
        (void) oneClient(c1, are1);
        print(PRI1, "mainflow2: oneClient(c2, are2);\n");
        (void) oneClient(c2, are2);
        print(PRI1, "mainflow2: oneClient(c3, are3);\n");
        (void) oneClient(c3, are3);

        print(PRI1, "mainflow2: co_await are1;\n");
        co_await are1;
        print(PRI1, "mainflow2: co_await are2;\n");
        co_await are2;
        print(PRI1, "mainflow2: co_await are3;\n");
        co_await are3;
        
        print(PRI1, "mainflow2: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    print(PRI1, "mainflow2: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

eager<int> oneClient(CommClient& c1)
{
    print(PRI2, "oneClient: begin\n");

    print(PRI2, "oneClient: async_operation sc1 = c1.start_connecting();\n");
    async_operation sc1 = c1.start_connecting();

    print(PRI2, "oneClient: co_await sc1;\n");
    co_await sc1;

    print(PRI2, "oneClient: async_operation sw1 = c1.start_writing(...);\n");
    async_operation sw1 = c1.start_writing("This is string 1 to echo\n", 
                                    strlen("This is string 1 to echo\n") + 1);

    print(PRI2, "oneClient: co_await sw1;\n");
    co_await sw1;

    print(PRI2, "oneClient: async_operation sr1 = c1.start_reading();\n");
    async_operation sr1 = c1.start_reading('\n');

    print(PRI2, "oneClient: co_await sr1;\n");
    CommService* ssr1 = co_await sr1;
    ssr1->get_result();

    print(PRI2, "oneClient: c1.stop();\n");
    c1.stop();

    co_return 0;
}

// -----------------------------------------------------------------

async_task<int> mainflow2a(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow2a: begin\n");
    for (int i = 0; i < 3; i++)
    {
        print(PRI2, "mainflow2a: -----------------------------------------\n");
        
        print(PRI1, "mainflow2a: oneClient(c1);\n");
        eager<int> tc1 = oneClient(c1);
        print(PRI1, "mainflow2a: oneClient(c2);\n");
        eager<int> tc2 = oneClient(c2);
        print(PRI1, "mainflow2: oneClient(c3);\n");
        eager<int> tc3 = oneClient(c3);

        print(PRI1, "mainflow2a: co_await tc1;\n");
        co_await tc1;
        print(PRI1, "mainflow2a: co_await tc1;\n");
        co_await tc2;
        print(PRI1, "mainflow2a: co_await tc1;\n");
        co_await tc3;

        print(PRI1, "mainflow2a: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    print(PRI1, "mainflow2a: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

async_task<int> mainflow_array(CommClient& c1, CommClient& c2, CommClient& c3)
{
    async_operation asyncsc[3];
    async_operation asyncsw[3];
    async_operation asyncsr[3];
    CommService* results[3];

    print(PRI2, "mainflow_array: begin\n");
    for (int i = 0; i < 3; i++)
    {
        int j;

        print(PRI2, "mainflow_array: asyncsc[0] = c1.start_connecting();\n");
        asyncsc[0] = c1.start_connecting();
        print(PRI2, "mainflow_array: asyncsc[1] = c2.start_connecting();\n");
        asyncsc[1] = c2.start_connecting();
        print(PRI2, "mainflow_array: asyncsc[2] = c3.start_connecting();\n");
        asyncsc[2] = c3.start_connecting();

        for (j = 0; j < 3; j++) {
            print(PRI2, "mainflow_array: co_await asyncsc[%d];\n", j);
            co_await asyncsc[j];
        }

        print(PRI2, "mainflow_array: asyncsw[0] = c1.start_writing();\n");
        asyncsw[0] = c1.start_writing("This is string 1 to echo\n", strlen("This is string 1 to echo\n") + 1);
        print(PRI2, "mainflow_array: asyncsw[1] = c2.start_writing();\n");
        asyncsw[1] = c2.start_writing("This is string 2 to echo\n", strlen("This is string 2 to echo\n") + 1);
        print(PRI2, "mainflow_array: asyncsw[2] = c3.start_writing();\n");
        asyncsw[2] = c3.start_writing("This is string 3 to echo\n", strlen("This is string 3 to echo\n") + 1);

        for (j = 0; j < 3; j++) {
            print(PRI2, "mainflow_array: co_await asyncsw[%d];\n", j);
            co_await asyncsw[j];
        }

        print(PRI2, "mainflow_array: asyncsr[0] = c1.start_reading();\n");
        asyncsr[0] = c1.start_reading();
        print(PRI2, "mainflow_array: asyncsr[1] = c2.start_reading();\n");
        asyncsr[1] = c2.start_reading();
        print(PRI2, "mainflow_array: asyncsr[2] = c3.start_reading();\n");
        asyncsr[2] = c3.start_reading();

        for (j = 0; j < 3; j++) {
            print(PRI2, "mainflow_array: results[%d] = co_await asyncsr[%d];\n", j, j);
            results[j] = co_await asyncsr[j];
        }

        for (j = 0; j < 3; j++) {
            print(PRI2, "mainflow_array: results[%d]->get_result()\n", j);
            results[j]->get_result();
        }

        print(PRI2, "mainflow_array: c1.stop();\n");
        c1.stop();
        print(PRI2, "mainflow_array: c2.stop();\n");
        c2.stop();
        print(PRI2, "mainflow_array: c3.stop();\n");
        c3.stop();

        print(PRI2, "mainflow_array: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print(PRI2, "mainflow_array: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

async_task<int> mainflow(std::initializer_list<CommClient*> clients)
{
    async_operation asyncsc[3];
    async_operation asyncsw[3];
    async_operation asyncsr[3];
    CommService* results[3];

    int sz = clients.size();

    print(PRI2, "mainflow: begin\n");
    for (int i = 0; i < 3; i++)
    {
        int j;
        
        j = 0;
        for (CommClient* cl: clients) {
            print(PRI2, "mainflow: asyncsc[%d] = cl->start_connecting();\n", j);
            asyncsc[j++] = cl->start_connecting();
        }
        for (j = 0; j < sz; j++) {
            print(PRI2, "mainflow: co_await asyncsc[%d];\n", j);
            co_await asyncsc[j];
        }

        j = 0;
        for (CommClient* cl : clients) {
            print(PRI2, "mainflow: asyncsw[%d] = cl->start_writing(...);\n", j);
            asyncsw[j++] = cl->start_writing("This is the string to echo\n", 28);
        }
        for (j = 0; j < sz; j++) {
            print(PRI2, "mainflow: co_await asyncsw[%d];\n", j);
            co_await asyncsw[j];
        }

        j = 0;
        for (CommClient* cl : clients) {
            print(PRI2, "mainflow: asyncsr[%d] = cl->start_reading();\n", j);
            asyncsr[j++] = cl->start_reading();
        }
        for (j = 0; j < sz; j++) {
            print(PRI2, "mainflow: results[%d] = co_await asyncsr[%d];\n", j, j);
            results[j] = co_await asyncsr[j];
        }

        for (j = 0; j < 3; j++) {
            print(PRI2, "mainflow: results[%d]->get_result()\n", j);
            results[j]->get_result();
        }

        for (CommClient* cl : clients) {
            print(PRI2, "mainflow: cl->stop();\n");
            cl->stop();
        }

        print(PRI2, "mainflow: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print(PRI2, "mainflow: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

void mainflowX(CommClient& c1, CommClient& c2, CommClient& c3, int selected)
{
    switch (selected) {
    case 0:
        {
            print(PRI2, "main: async_task<int> si1 = mainflow(c1, c2, c3);\n");
            async_task<int> si1 = mainflow(c1, c2, c3);
        }
        break;
    case 1:
        {
            print(PRI2, "main: async_task<int> si1 = mainflow2(c1, c2, c3);\n");
            async_task<int> si1 = mainflow2(c1, c2, c3);
        }
        break;
    case 2:
        {
            print(PRI2, "main: async_task<int> si1 = mainflow2a(c1, c2, c3);\n");
            async_task<int> si1 = mainflow2a(c1, c2, c3);
        }
        break;
    case 3:
        {
            print(PRI2, "main: async_task<int> si2 = mainflow_array(c1, c2, c3);\n");
            async_task<int> si2 = mainflow_array(c1, c2, c3);
        }
        break;
    case 4:
        {
            print(PRI2, "main: async_task<int> si3 = mainflow( {&c1, &c2, &c3} )\n");
            async_task<int> si3 = mainflow({ &c1, &c2, &c3 });
        }
        break;
    }
}

// -----------------------------------------------------------------

async_task<int> mainflowAll(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflowAll: async_task<int> si0 = mainflow(c1, c2, c3);\n");
    async_task<int> si0 = mainflow(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si0;\n");
    co_await si0;

    print(PRI1, "mainflowAll: async_task<int> si1 = mainflow2(c1, c2, c3)\n");
    async_task<int> si1 = mainflow2(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si1;\n");
    co_await si1;
	
    print(PRI1, "mainflowAll: async_task<int> si2 = mainflow2a(c1, c2, c3)\n");
    async_task<int> si2 = mainflow2a(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si2;\n");
    co_await si2;

    print(PRI1, "mainflowAll: async_task<int> si3 = mainflow_array(c1, c2, c3)\n");
    async_task<int> si3 = mainflow_array(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si3;\n");
    co_await si3;

    print(PRI1, "mainflowAll: async_task<int> si4 = mainflow( {&c1, &c2, &c3} )\n");
    async_task<int> si4 = mainflow({ &c1, &c2, &c3 });
    print(PRI1, "mainflowAll: co_await si4;\n");
    co_await si4;

    print(PRI1, "mainflowAll: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

int main(int argc, char* argv[])
{
    priority = 0x0F;

    boost::asio::io_context ioContext;

    print(PRI2, "main: CommClient c(ioContext);\n");
    CommClient c1(ioContext, ep);
    CommClient c2(ioContext, ep);
    CommClient c3(ioContext, ep);

    if (argc == 2)
    {
        int selected = 2;
        selected = atoi(argv[1]);
        if (selected < 0 || selected > 4)
        {
            print(PRI1, "main: selection must be in the range [0..3]\n");
            return 0;
        }
        print(PRI1, "main: mainflowX(c1, c2, c3, selected);\n");
        mainflowX(c1, c2, c3, selected);
    }
    else
    {
        print(PRI1, "main: async_task<int> si = mainflowAll(c1, c2, c3);\n");
        async_task<int> si = mainflowAll(c1, c2, c3);
    }

    print(PRI2, "main: ioContext.run();\n");
    ioContext.run();

    print(PRI2, "main: return 0;\n");
    return 0;
}

// -----------------------------------------------------------------

