/** 
 *  Filename: p0800cs.cpp
 *  Description:
 *        This echo server used is for p07XX.cpp and p08XX.cpp clients.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <boost/asio.hpp>

#include <signal.h>
#include <boost/asio/signal_set.hpp>

#include <thread>
#include <string>

const int priority = 0x01;

#include "print.h"
#include "csemaphore.h"

// -----------------------------------------------------------------
// Auxiliary awaitable that allows a coroutine to return control to main().
// Does not define await_ready, await_suspend and await_resume (not needed)
// because main() will (can) not co_await an eager object.

#include <atomic>
#include <coroutine>
#include <type_traits>
#include <cassert>
#include <exception>

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

#if 1
    // Alternative 1: define operator co_await and an awaiter type
    // that defines await_ready(), await_suspend() and await_resume().

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
#else
    // Alternative 2: define await_ready(), await_suspend() and await_resume()
    // in the coroutine type.

    bool await_ready() {
        const bool ready = coro.done();
        print(PRI2, "%p: eager::await_ready(): return %d;\n", this, ready);
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print(PRI2, "%p: eager::await_suspend(std:coroutine_handle<> awaiting)\n", this);
        coro.promise().m_awaiting = awaiting;
    }

    T await_resume() {
        print(PRI2, "%p: eager::await_resume()\n", this);
        const T r = coro.promise().m_value;
        return r;
    }
#endif

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
            print(PRI2, "%p: eager::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        CSemaphore m_sema;
        std::coroutine_handle<> m_awaiting;
        bool m_wait_for_signal;
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
    static const int NROPERATIONS = 128;
    CommService()
        : index(-1) 
    {
        for (int i = 0; i < NROPERATIONS; i++)
            m_async_operations[i] = nullptr;
    }

    int index = 0;
    async_operation* m_async_operations[NROPERATIONS];
};

// -----------------------------------------------------------------

class async_operation {
public:
    async_operation(CommService* s = nullptr, int index = 0)
        : m_service(s)
        , m_awaiting(nullptr)
        , m_ready(false)
        , m_waiting_coroutine(false)
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
        , m_ready(s.m_ready)
        , m_waiting_coroutine(s.m_waiting_coroutine)
    {
        print(PRI2, "%p: async_operation::async_operation(eager&& s)\n", this);
        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        s.m_waiting_coroutine = false;
    }

    async_operation& operator = (const async_operation&) = delete;

    async_operation& operator = (async_operation&& s) {
        print(PRI2, "%p: async_operation::async_operation = (async_operation&& s)\n", this);
        m_service = s.m_service;
        m_awaiting = s.m_awaiting;
        m_waiting_coroutine = s.m_waiting_coroutine;
       
        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_waiting_coroutine = false;
        return *this;
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
                print(PRI2, "%p: async_operation::await_ready(): return %d;\n", &m_async, m_async.m_ready);
                return m_async.m_ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print(PRI2, "%p: async_operation::await_suspend(...)\n", &m_async);
                m_async.m_awaiting = awaiting;
                m_async.m_waiting_coroutine = true;
            }

            void await_resume() {
                print(PRI1, "%p: async_operation::await_resume()\n", &m_async);
            }

        private:
            async_operation& m_async;
        };

        return awaiter{ *this };
    }

private:
    CommService* m_service;
    std::coroutine_handle<> m_awaiting;
    bool m_ready;
    bool m_waiting_coroutine;
};

// -----------------------------------------------------------------

std::atomic_bool stop{false};

std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred)
{
    static bool printmsg = true;
    static int counter = 0;

    if (bytes_transferred > 0) {
        counter = 0;
        printmsg = true;
    }
        
    if (printmsg && (counter++ < 100))
        print(PRI2, "completionCondition(..., bytes_transferred = %ld)\n", bytes_transferred);

    if (!bytes_transferred)
    {
        return 1;
    }
    return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
}

// -----------------------------------------------------------------

struct Clientx
{
    explicit Clientx(boost::asio::io_context& ioContext) :
        mReadBuffer{},
        mSendBuffer{},
        mSocket{ioContext}
    {
        print(PRI2, "%p: Clientx::Clientx()\n", this);
    }

    ~Clientx()
    {
        print(PRI2, "%p: Clientx::~Clientx()\n", this);
    }

    void close()
    {
        print(PRI2, "%p: Clientx::close()\n", this);
        mSocket.close();
    }

    std::string mReadBuffer;
    std::string mSendBuffer;
    boost::asio::ip::tcp::socket mSocket;
};

// -----------------------------------------------------------------

using ClientSession = std::shared_ptr<Clientx>;

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

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

const boost::asio::ip::tcp::endpoint ep1{ boost::asio::ip::make_address("127.0.0.1"), 8242 };
const boost::asio::ip::tcp::endpoint ep2{ boost::asio::ip::make_address("127.0.0.1"), 8342 };

using boost::asio::steady_timer;
using boost::asio::ip::tcp;


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

    async_operation start_timer(steady_timer& timer, int ms)
    {
        print(PRI1, "%p: CommClient::start_timer(timer, %d)\n", this, ms);
        index = (index + 1) % NROPERATIONS;
        start_tmr(index, timer, ms);
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

    void start_write(const int idx, const char* str, int size)
    {
        print(PRI2, "%p: CommClient::start_write()\n", this);

        if (stopped_)
            return;

        boost::asio::async_write(
            socket_,
            boost::asio::buffer(str, size),
            [this, idx](const boost::system::error_code& error,
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

    void start_tmr(const int idx, steady_timer& tmr, int ms)
    {
        print(PRI2, "%p: CommClient::start_tmr()\n", this);

        tmr.expires_after(std::chrono::milliseconds(ms));
        tmr.async_wait(
            [this, idx](const boost::system::error_code& error)
            {
                print(PRI1, "%p: CommClient::handle_timer()\n", this);
                async_operation* om_async_operation = m_async_operations[idx];

                if (stopped_)
                    return;

                if (!error)
                {
                    print(PRI2, "%p: CommClient::handle_timer(): om_async_operation = %p\n", this, om_async_operation);
                    if (om_async_operation)
                    {
                        om_async_operation->completed();
                    }
                }
                m_async_operations[idx] = nullptr;
                print(PRI2, "%p: CommClient::handle_timer(): exit\n\n", this);
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

class Server : public CommService
{
public:
    Server(
            boost::asio::io_context& ioContext,
            unsigned short CommService) :
        mIoContext{ioContext},
        mAcceptor{mIoContext, {boost::asio::ip::tcp::v4(), CommService}},
        mStop{false}
    {
        print(PRI2, "Server::Server(...)\n");
    }

    async_operation start_accepting(ClientSession CommClient)
    {
        print(PRI1, "%p: CommClient::start_accepting()\n", this);
        index = (index + 1) % 128;
        start_accept(CommClient, index);
        return async_operation{ this, index};
    }

    async_operation start_writing(ClientSession CommClient)
    {
        print(PRI1, "%p: CommClient::start_writing()\n", this);
        index = (index + 1) % 128;
        start_write(CommClient, index);
        return async_operation{ this, index };
    }

    async_operation start_reading(ClientSession CommClient)
    {
        print(PRI1, "%p: CommClient::start_reading()\n", this);
        index = (index + 1) % 128;
        start_read(CommClient, index);
        return async_operation{ this, index };
    }

public:

    // -----------------------------------------------------

    void stop()
    {
        print(PRI2, "Server::stop()\n");
        mStop = true;
        mAcceptor.cancel();
    }

    void start_accept(ClientSession CommClient, int idx)
    {
        print(PRI2, "Server::start_accept()\n");

        mAcceptor.async_accept(
            CommClient->mSocket,
            [this, idx](const boost::system::error_code& ec) {
                print(PRI1, "Server::acceptHandler(): entry: idx = %d\n", idx);
                async_operation* om_async_operation = m_async_operations[idx];

                if (mStop)
                {
                    return;
                }
                if (ec)
                {
                    print(PRI2, "Server::acceptHandler(...): accept failed: %s\n", ec.message().c_str());
                }
                else
                {
                    print(PRI2, "Server::acceptHandler(...): om_async_operation = %p\n", om_async_operation);
                    if (om_async_operation)
                    {
                        print(PRI2, "Server::acceptHandler(...): before om_async_operation->completed();\n");
                        om_async_operation->completed();
                        print(PRI2, "Server::acceptHandler(...): after om_async_operation->completed();\n");
                    }
                }
                m_async_operations[idx] = nullptr;
                print(PRI1, "Server::acceptHandler(): exit\n\n");
            }
        );
    }

    void start_read(ClientSession CommClient, int idx)
    {
        print(PRI2, "Server::start_read(ClientSession CommClient)\n");

        boost::asio::async_read_until(
            CommClient->mSocket,
            boost::asio::dynamic_buffer(CommClient->mReadBuffer),
            '\n',
            [CommClient, this, idx](const boost::system::error_code& error,
                std::size_t bytes)
            {
                print(PRI1, "Server::readhandler(): entry, idx = %d\n", idx);
                async_operation* om_async_operation = m_async_operations[idx];

                if (mStop)
                {
                    return;
                }

                if (!error)
                {
                    print(PRI2, "Server::readHandler(...): read %d bytes\n", bytes);
                    print(PRI2, "Server::readHandler(...): std::copy\n");

                    print(PRI2, "Server::readHandler(...): read %s\n", CommClient->mReadBuffer.c_str());

                    CommClient->mSendBuffer.resize(bytes);
                    print(PRI2, "Server::readHandler(...): std::copy\n", bytes);

                    std::copy(CommClient->mReadBuffer.cbegin(), 
                              CommClient->mReadBuffer.cbegin() + bytes, 
                              CommClient->mSendBuffer.begin());
                    print(PRI2, "Server::readHandler(...): %s\n", CommClient->mSendBuffer.c_str());

                    print(PRI2, "Server::readHandler(...): om_async_operation = %p\n", om_async_operation);
                    if (om_async_operation)
                    {
                        print(PRI2, "Server::readHandler(...): before om_async_operation->completed();\n");
                        om_async_operation->completed();
                        print(PRI2, "Server::readHandler(...): after om_async_operation->completed();\n");
                    }
                }
                else
                {
                    print(PRI2, "Server::readHandler(): Error on receive: %s\n", error.message().c_str());
                }
                m_async_operations[idx] = nullptr;
                print(PRI1, "Server::readHandler(): exit\n\n");
        });
    }

    void start_write(ClientSession CommClient, int idx)
    {
        print(PRI2, "Server::start_write(ClientSession CommClient)\n");

        boost::asio::async_write(
            CommClient->mSocket,
            boost::asio::buffer(CommClient->mSendBuffer),
            [this, idx](const boost::system::error_code& error,
                std::size_t /*result_n*/)
            {
                print(PRI1, "Server::writehandler(): entry, idx = %d\n", idx);
                async_operation* om_async_operation = m_async_operations[idx];

                if (!error)
                {
                    print(PRI2, "Server::writehandler(): om_async_operation = %p\n", om_async_operation);
                    if (om_async_operation)
                    {
                        print(PRI2, "Server::writehandler(...): before om_async_operation->completed();\n");
                        om_async_operation->completed();
                        print(PRI2, "Server::writehandler(...): after om_async_operation->completed();\n");
                    }
                }
                else
                {
                    print(PRI2, "Server::writehandler(): Error on write: %s\n", error.message().c_str());
                    //stop();
                }
                m_async_operations[idx] = nullptr;
                print(PRI1, "Server::writehandler(): exit\n\n");
            });
    }

protected:
    boost::asio::io_context& mIoContext;
    boost::asio::ip::tcp::acceptor mAcceptor;
    std::atomic_bool mStop;
};

// -----------------------------------------------------------------

boost::asio::io_context ioContextSignal;
boost::asio::io_context ioContextServer;

// -----------------------------------------------------------------

oneway_task mainflow_client()
{
    CommClient c(ioContextServer, ep2);

    print(PRI2, "mainflow_client: begin\n");

    for (int i = 0; i < 1; i++)
    {
        print(PRI2, "mainflow_client: async_operation sc = start_connecting();\n");
        async_operation sc = c.start_connecting();
        print(PRI2, "mainflow_client: co_await sc;\n");
        co_await sc;

        print(PRI2, "mainflow_client: async_operation sw = c.start_writing(...);\n");
        async_operation sw = c.start_writing("This is string 1 to echo\n", strlen("This is string 1 to echo\n") + 1);

        print(PRI2, "mainflow_client: co_await sw;\n");
        co_await sw;

        print(PRI2, "mainflow_client: async_operation sr = c.start_reading();\n");
        async_operation sr = c.start_reading();
        print(PRI2, "mainflow_client: co_await sr;\n");
        co_await sr;

        steady_timer client_timer(ioContextServer);
        print(PRI1, "mainflow_client: async_operation st = c.start_timer(client_timer, 2000);\n");
        async_operation st = c.start_timer(client_timer, 2000);
        print(PRI1, "mainflow_client: co_await st;\n");
        co_await st;

        print(PRI1, "mainflow_client: stop();\n");
        c.stop();
    }

    print(PRI2, "mainflow_client: co_return;\n");
}

// -----------------------------------------------------------------

class ServerApp : public Server
{
public:
    ServerApp(
        boost::asio::io_context& ioContext,
        unsigned short CommService) :
        Server(ioContext, CommService)
    {
        print(PRI2, "ServerApp::ServerApp(...)\n");
    }

    oneway_task mainflow_reading_writing(ClientSession CommClient)
    {
        print(PRI2, "mainflow_reading_writing: async_operation sr = start_reading(CommClient);\n");
        async_operation sr = start_reading(CommClient);

        print(PRI2, "mainflow_reading_writing: co_await sr;\n");
        co_await sr;

        print(PRI2, "mainflow_reading_writing: mainflow_client();\n");
        (void) mainflow_client();

        print(PRI2, "mainflow_reading_writing: async_operation sw = start_writing(CommClient);\n");
        async_operation sw = start_writing(CommClient);

        print(PRI2, "mainflow_reading_writing: co_await sw;\n");
        co_await sw;

        print(PRI2, "mainflow_reading_writing: co_return\n");
    }

    eager<int> mainflow()
    {
        while (1)
        {
            print(PRI2, "mainflow: -------------------------------------------------\n");

            auto CommClient = std::make_shared<Clientx>(mIoContext);

            print(PRI2, "mainflow: async_operation sa = start_accepting(CommClient);\n");
            async_operation sa = start_accepting(CommClient);

            print(PRI2, "mainflow: co_await sa;\n");
            co_await sa;

            print(PRI2, "mainflow: mainflow_reading_writing(CommClient);\n");
            (void) mainflow_reading_writing(CommClient);
        }

        print(PRI2, "mainflow: co_return 0;\n");
        co_return 0;
    }

};

// -----------------------------------------------------------------

void runServer(
    ServerApp& server,
    boost::asio::io_context& ioContext)
{
    print(PRI2, "runServer(...): eager<int> si = server.mainflow();\n");
    eager<int> si = server.mainflow();

    print(PRI2, "runServer(...): ioContext.run();\n");
    ioContext.run();
}

void handlerSignals(
        const boost::system::error_code& /*erc*/,
        int signal)
{
    print(PRI2, "handlerSignals(...): signal %d occurred. Time to stop the application", signal);
    stop = true;
}

void asyncSignal(boost::asio::io_context& ioContext)
{
    print(PRI2, "asyncSignal(...)\n");
    boost::asio::signal_set signals{ioContext, SIGINT, SIGTERM};

    signals.async_wait(handlerSignals);
    ioContext.run();
}

// -----------------------------------------------------------------

int main()
{
    //boost::asio::io_context ioContextSignal;
    //boost::asio::io_context ioContextServer;

    //CommClient c1(ioContextServer, ep1);
    //eager<int> si = mainflow(c1);

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

    ServerApp server1{ioContextServer, 8242};

    std::thread t1{ runServer, std::ref(server1), std::ref(ioContextServer) };

    t.join(); /// wait on stop signal

    print(PRI2, "main(): server1.stop()\n");
    server1.stop();
    t1.join();

    print(PRI2, "main(): return 0;\n");
    return 0;
}
