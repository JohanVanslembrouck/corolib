/** 
 *  Filename: p0800s.cpp
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

const int priority = 0x0F;

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

    eager() {
        print(PRI2, "%p: eager::eager()\n", this);
    }

    ~eager() {
        print(PRI2, "%p: eager::~eager()\n", this);
        if (coro && coro.done())
            coro.destroy();
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
protected:
    CommService()
        : index(-1) 
    {
        for (int i = 0; i < 128; i++)
            m_async_operations[i] = nullptr;
    }

    int index = 0;
    async_operation* m_async_operations[128];
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
        m_service->m_async_operations[index] = this;
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
                print(PRI2, "%p: async_operation::await_resume()\n", &m_async);
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
        print(PRI2, "%p: Server::start_accepting()\n", this);
        index = (index + 1) % 128;
        start_accept(CommClient, index);
        return async_operation{ this, index};
    }

    async_operation start_writing(ClientSession CommClient)
    {
        print(PRI2, "%p: Server::start_writing()\n", this);
        index = (index + 1) % 128;
        start_write(CommClient, index);
        return async_operation{ this, index };
    }

    async_operation start_reading(ClientSession CommClient)
    {
        print(PRI2, "%p: Server::start_reading()\n", this);
        index = (index + 1) % 128;
        start_read(CommClient, index);
        return async_operation{ this, index };
    }

    async_operation start_timer(boost::asio::steady_timer& timer, int ms)
    {
        print(PRI1, "%p: CommClient::start_timer(timer, %d)\n", this, ms);
        index = (index + 1) % 128;
        start_tmr(timer, ms, index);
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
                print(PRI2, "Server::acceptHandler(): entry: idx = %d\n", idx);
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
                print(PRI2, "Server::acceptHandler(): exit\n\n");
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
                print(PRI2, "Server::readhandler(): entry, idx = %d\n", idx);
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
                print(PRI2, "Server::readHandler(): exit\n\n");
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
                print(PRI2, "Server::writehandler(): entry, idx = %d\n", idx);
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
                    print(PRI2, "Server::writehandler(...) Error on write: %s\n", error.message().c_str());
                }
                print(PRI2, "Server::writehandler(...): exit\n\n");
            });
    }

    void start_tmr(boost::asio::steady_timer& tmr, int ms, int idx)
    {
        print(PRI2, "Server::start_tmr()\n", this);

        tmr.expires_after(std::chrono::milliseconds(ms));
        tmr.async_wait(
            [this, idx](const boost::system::error_code& error)
            {
                print(PRI2, "Server::handle_timer(): entry, idx = %d\n", idx);
                async_operation* om_async_operation = m_async_operations[idx];

                if (!error)
                {
                    print(PRI2, "Server::handle_timer(): om_async_operation = %p\n", om_async_operation);
                    if (om_async_operation)
                    {
                        print(PRI2, "Server::handle_timer(...): before om_async_operation->completed();\n");
                        om_async_operation->completed();
                        print(PRI2, "Server::handle_timer(...): after om_async_operation->completed();\n");
                    }
                }
            });
    }

protected:
    boost::asio::io_context& mIoContext;
    boost::asio::ip::tcp::acceptor mAcceptor;
    std::atomic_bool mStop;
};

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

        print(PRI2, "mainflow_reading_writing: async_operation sw = start_writing(CommClient);\n");
        async_operation sw = start_writing(CommClient);

        print(PRI2, "mainflow_reading_writing: co_await sw;\n");
        co_await sw;

        boost::asio::steady_timer client_timer(mIoContext);
        print(PRI1, "mainflow_reading_writing: async_operation st = c.start_timer(client_timer, 100);\n");
        async_operation st = start_timer(client_timer, 100);

        print(PRI1, "mainflow_reading_writing: co_await st;\n");
        co_await st;

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
    boost::asio::io_context ioContextSignal;
    boost::asio::io_context ioContextServer;

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

    //ServerApp server1{ ioContextServer, 8242 };
    ServerApp server1{ ioContextServer, 8342 };

    std::thread t1{ runServer, std::ref(server1), std::ref(ioContextServer) };

    t.join(); /// wait on stop signal

    print(PRI2, "main(): server1.stop()\n");
    server1.stop();
    t1.join();

    print(PRI2, "main(): return 0;\n");
    return 0;
}
