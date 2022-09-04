/**
 *  Filename: p0830c.cpp
 *  Description:
 *        This example illustrates the use of coroutines
 *        in combination with Boost ASIO to implement an echo client.
 *
 *        This example uses 3 clients.
 *        It also uses a wait_all_awaitable type
 *        that allows awaiting the completion of N asychronous operations.
 *
 *        Each client connects with a server and then reads several replies
 *        from that server.
 *        This example is an attempt to implement a simple publish-subscribe system.
 *        - subscription is implemented by just connecting to the server
 *        - the server will then (regularly) publish information to its subscribed clients
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon: cppcoro implementation of when_all_ready.
 *
 */

#include <boost/asio.hpp>

#include <thread>
#include <iostream>
#include <string>

const boost::asio::ip::tcp::endpoint ep{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

// -----------------------------------------------------------------

#include <experimental/resumable>

#include <mutex>
#include <condition_variable>

class CSemaphore
{
private:
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned int count_;
public:
    CSemaphore() : count_() { }

    void reset() {
        std::unique_lock<std::mutex> lock(mutex_);
        count_ = 0;
    }

    void signal() {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait() {
        std::unique_lock < std::mutex > lock(mutex_);
        while (!count_)
            condition_.wait(lock);
        --count_;
    }
};

// -----------------------------------------------------------------

uint64_t threadids[128];

int get_thread_number(uint64_t id)
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
    //static_assert(sizeof(std::thread::id) == sizeof(uint64_t), "this function only works if size of thead::id is equal to the size of uint_64");
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)& id;
    return (*ptr);
}

void print(const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = get_thread_number(get_thread_id());
    fprintf(stderr, "%02d: %s", threadid, msg);
}

// -----------------------------------------------------------------

template<typename T>
struct async_task {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    async_task(const async_task& s) = delete;

    async_task(async_task&& s)
        : coro(s.coro) {
        print("%p: async_task::async_task(async_task&& s)\n", this);
        s.coro = nullptr;
    }

    ~async_task() {
        print("%p: async_task::~async_task()\n", this);
        //if (coro) coro.destroy();    // can throw exception when async_task goes out of scope. FFS
    }

    async_task(handle_type h)
        : coro(h) {
        print("%p: async_task::async_task(handle_type h)\n", this);
    }

    async_task& operator = (const async_task&) = delete;

    async_task& operator = (async_task&& s) {
        print("%p: async_task::async_task = (async_task&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("%p: async_task::get()\n", this);
        if (!coro.promise().m_ready) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().sema.wait();
        }
        return coro.promise().value;
    }

    struct promise_type {

        friend struct async_task;

        promise_type() :
            m_awaiting(nullptr),
            m_ready(false),
            m_wait_for_signal(false),
            m_waiting_coroutine(false) {
            print("%p: async_task::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: async_task::promise_type::~promise_type()\n", this);
        }

        auto return_value(T v) {
            print("%p: async_task::promise_type::return_value(T v): begin\n", this);
            value = v;
            m_ready = true;
            if (m_waiting_coroutine) {
                print("%p: async_task::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print("%p: async_task::promise_type::return_value(T v): after m_awaiting.resume();\n", this);
            }
            if (m_wait_for_signal) {
                print("%p: async_task::promise_type::return_value(T v): before sema.signal();\n", this);
                sema.signal();
                print("%p: async_task::promise_type::return_value(T v): after sema.signal();\n", this);
            }
            print("%p: async_task::promise_type::return_value(T v): end\n", this);
        }

        auto get_return_object() {
            print("%p: async_task::promise_type::get_return_object()\n", this);
            return async_task<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("%p: async_task::promise_type::initial_suspend()\n", this);
            return std::experimental::suspend_never{};
        }

        auto final_suspend() {
            print("%p: async_task::promise_type::final_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: async_task::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T value;
        bool m_ready;
        CSemaphore sema;
        bool m_wait_for_signal;
        std::experimental::coroutine_handle<> m_awaiting;
        bool m_waiting_coroutine;
    };

    handle_type coro;
};

// -----------------------------------------------------------------
// -----------------------------------------------------------------
// Core library comprising the base class 'service' and
// awaitable type async_operation.

struct async_operation;

class service {
    friend struct async_operation;
protected:
    async_operation* m_async_operation;
};

// -----------------------------------------------------------------

struct wait_all_counter
{
    std::experimental::coroutine_handle<> m_awaiting;

    wait_all_counter(int nr) :
        m_nr(nr)
    {
        print("%p: wait_all_counter::wait_all_counter(%d)\n", this, nr);
    }

    void completed() {
        print("%p: wait_all_counter::completed(): m_nr = %d\n", this, m_nr);
        m_nr--;
        if (m_nr == 0) {
            print("%p: wait_all_counter::completed(): m_nr = 0!!!\n", this, m_nr);
            m_awaiting.resume();
        }
    }

    int m_nr;
};

// -----------------------------------------------------------------

struct async_operation {

    async_operation(service* s = nullptr)
        : m_service(s)
        , m_awaiting(nullptr)
        , m_waiting_coroutine(false)
        , m_ready(false)
    {
        print("%p: async_operation::async_operation(service* s = %p)\n", this, s);
        if (m_service)
            m_service->m_async_operation = this;
    }

    ~async_operation() {
        print("%p: async_operation::~async_operation()\n", this);
    }

    async_operation(const async_operation& s) = delete;

    async_operation(async_operation&& s)
        : m_service(s.m_service)
        , m_awaiting(s.m_awaiting)
        , m_waiting_coroutine(s.m_waiting_coroutine)
        , m_ready(s.m_ready)
    {
        print("%p: async_operation::async_operation(async_operation&& s = %p)\n", this, &s);

        print("%p: async_operation::async_operation(async_operation&& s): m_service = %p\n", this, m_service);
        print("%p: async_operation::async_operation(async_operation&& s): m_awaiting = %p\n", this, m_awaiting);
        print("%p: async_operation::async_operation(async_operation&& s): m_waiting_coroutine = %d\n", this, m_waiting_coroutine);
        print("%p: async_operation::async_operation(async_operation&& s): m_ready = %d\n", this, m_ready);

        // Tell the service we are at another address after the move.
        m_service->m_async_operation = this;

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_waiting_coroutine = false;
        s.m_ready = false;
    }

    async_operation& operator = (const async_operation&) = delete;

    async_operation& operator = (async_operation&& s) {
        print("%p: async_operation::operator = (async_operation&& s = %p)\n", this, &s);

        m_service = s.m_service;
        m_awaiting = s.m_awaiting;
        m_waiting_coroutine = s.m_waiting_coroutine;
        m_ready = s.m_ready;
        // Tell the service we are at another address after the move.
        m_service->m_async_operation = this;

        print("%p: async_operation::operator = (async_operation&& s): m_service = %p\n", this, m_service);
        print("%p: async_operation::operator = (async_operation&& s): m_awaiting = %p\n", this, m_awaiting);
        print("%p: async_operation::operator = (async_operation&& s): m_waiting_coroutine = %d\n", this, m_waiting_coroutine);
        print("%p: async_operation::operator = (async_operation&& s): m_ready = %d\n", this, m_ready);

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_waiting_coroutine = false;
        s.m_ready = false;
        return *this;
    }

    void completed() {
        if (m_waiting_coroutine) {
            print("%p: async_operation::completed(): before m_awaiting.resume();\n", this);
            m_awaiting.resume();
            m_ready = true;
            print("%p: async_operation::completed(): after m_awaiting.resume();\n", this);
        }
        else if (m_ctr) {
            print("%p: async_operation::completed(): before m_ctr->completed();\n", this);
            m_ctr->completed();
            print("%p: async_operation::completed(): after m_ctr->completed();\n", this);
        }
        else {
            print("%p: async_operation::completed(): m_awaiting not yet initialized!!!\n", this);
        }
    }

    void get() {
        print("%p: async_operation::get()\n", this);
    }

    void setCounter(wait_all_counter *ctr)
    {
        print("%p: void async_operation::setCounter(%p)\n", this, ctr);
        m_ctr = ctr;
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(async_operation& async_) : 
                m_async(async_) 
            {
            }

            bool await_ready() {
                print("%p: async_operation::await_ready(): return %d;\n", this, m_async.m_ready);
                return m_async.m_ready;
            }

            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
                print("%p: async_operation::await_suspend(...)\n", this);
                m_async.m_awaiting = awaiting;
                m_async.m_waiting_coroutine = true;
            }

            void await_resume() {
                print("%p: async_operation::await_resume()\n", this);
            }

        private:
            async_operation& m_async;
        };

        return awaiter{ *this };
    }

private:
    service* m_service;
    std::experimental::coroutine_handle<> m_awaiting;
    bool m_waiting_coroutine;
    bool m_ready;
    wait_all_counter* m_ctr;
};

// -----------------------------------------------------------------

template<typename T>
struct wait_all_awaitable {

    wait_all_counter m_counter;

    wait_all_awaitable(std::initializer_list<async_operation*> aws)
        : m_counter(aws.size())
    {
        print("%p: wait_all_awaitable::wait_all_awaitable(std::initializer_list<async_operation*> aws)\n", this);
        for (async_operation* a : aws)
        {
            a->setCounter(&m_counter);
        }
    }
    
    wait_all_awaitable(async_operation* aws, int size)
        : m_counter(size)
    {
        print("%p: wait_all_awaitable::wait_all_awaitable(sync_operation* aws, int size)\n", this);
        for (int i = 0; i < size; i++)
        {
            aws[i].setCounter(&m_counter);
        }
    }

    wait_all_awaitable(const wait_all_awaitable& s) = delete;

    wait_all_awaitable(wait_all_awaitable&& s) {
        print("%p: wait_all_awaitable::wait_all_awaitable(async_task&& s)\n", this);
        //s.coro = nullptr;
    }

    ~wait_all_awaitable() {
        print("%p: async_task::~async_task()\n", this);
        //if (coro) coro.destroy();    // can throw exception when async_task goes out of scope. FFS
    }

    wait_all_awaitable& operator = (const wait_all_awaitable&) = delete;

    wait_all_awaitable& operator = (wait_all_awaitable&& s) {
        print("%p: wait_all_awaitable::wait_all_awaitable = (wait_all_awaitable&& s)\n", this);
        s.coro = nullptr;
        return *this;
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(wait_all_awaitable& sync_) : m_sync(sync_) {}

            bool await_ready() {
                bool ready = (m_sync.m_counter.m_nr == 0);
                print("%p: wait_all_awaitable::await_ready(): return %d;\n", this, ready);
                return ready;
            }

            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
                print("%p: wait_all_awaitable::await_suspend(...)\n", this);
                m_sync.m_counter.m_awaiting = awaiting;
            }

            T await_resume() {
                print("%p: wait_all_awaitable::await_resume()\n", this);
                //const T r = m_sync.coro.promise().value;
                const T r = 0;
                return r;
            }

        private:
            wait_all_awaitable& m_sync;
        };

        return awaiter{ *this };
    }
};

// -----------------------------------------------------------------
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

class client : public service
{
public:
    client(boost::asio::io_context& io_context, 
           boost::asio::ip::tcp::endpoint ep)
        : socket_(io_context),
        deadline_(io_context),
        heartbeat_timer_(io_context),
        m_ep(ep)
    {
        print("%p: client::client()\n", this);
    }

    void start()
    {
        print("%p: client::start()\n", this);
        start_connect();

        deadline_.async_wait(std::bind(&client::check_deadline, this));
    }

    void stop()
    {
        print("%p: client::stop()\n", this);
        stopped_ = true;
        boost::system::error_code ignored_error;
        socket_.close(ignored_error);
        deadline_.cancel();
        heartbeat_timer_.cancel();
    }

    async_operation start_connecting()
    {
        print("%p: client::start_connecting()\n", this);
        start_connect();
        return async_operation{ this };
    }

    async_operation start_reading()
    {
        print("%p: client::start_reading()\n", this);
        start_read();
        return async_operation{ this };
    }

private:
    void start_connect()
    {
        print("%p: client::start_connect(): m_async_operation = %p\n", this, m_async_operation);

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
            [&](const boost::system::error_code& error,
                const tcp::endpoint& /*result_endpoint*/)
            {
                print("%p: client::handle_connect(): entry\n", this);

                if (stopped_)
                    return;

                // The async_operation() function automatically opens the socket at the start
                // of the asynchronous operation. If the socket is closed at this time then
                // the timeout handler must have run first.
                if (!socket_.is_open())
                {
                    print("%p: client::handle_connect(): Connect timed out\n", this);

                    // Try the next available endpoint.
                    start_connect();
                }

                // Check if the connect operation failed before the deadline expired.
                else if (error)
                {
                    print("%p: client::handle_connect(): Connect error: %d\n", this, error);

                    // We need to close the socket used in the previous connection attempt
                    // before starting a new one.
                    socket_.close();

                    // Try the next available endpoint.
                    start_connect();
                }

                // Otherwise we have successfully established a connection.
                else
                {
                    print("%p: client::handle_connect(): successful connection: m_async_operation = %p\n",
                                this, m_async_operation);
                    if (m_async_operation)
                    {
                        m_async_operation->completed();
                    }
                }
                print("%p: client::handle_connect(): exit\n", this);
            });
    }

    void start_read()
    {
        print("%p: client::start_read(): m_async_operation = %p\n", this, m_async_operation);

        // Set a deadline for the read operation.
        deadline_.expires_after(std::chrono::seconds(10));

        boost::asio::async_read_until(
            socket_,
            boost::asio::dynamic_buffer(input_buffer_), '\n',
            [&](const boost::system::error_code& error,
                std::size_t n)
            {
                print("%p: client::handle_read(): entry\n", this);

                if (stopped_)
                    return;

                if (!error)
                {
                    // Extract the newline-delimited message from the buffer.
                    std::string line(input_buffer_.substr(0, n - 1));
                    input_buffer_.erase(0, n);

                    // Empty messages are heartbeats and so ignored.
                    if (!line.empty())
                    {
                        print("%p: Received: %s\n", this, line.c_str());
                    }

                    print("%p: client::handle_read(): successful read: m_async_operation = %p\n",
                        this, m_async_operation);
                    if (m_async_operation)
                    {
                        //m_async_operation->completed();
                        start_read();
                    }
                }
                else
                {
                    print("%p: Error on receive: %s\n", this, error.message().c_str());
                    m_async_operation->completed();
                    //stop();
                }
                print("%p: client::handle_read(): exit\n", this);
            });

    }

    void check_deadline()
    {
        print("%p: client::check_deadline()\n", this);

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
        deadline_.async_wait(std::bind(&client::check_deadline, this));
    }

private:
    bool stopped_ = false;
    //tcp::resolver::results_type endpoints_;
    tcp::socket socket_;
    std::string input_buffer_;
    steady_timer deadline_;
    steady_timer heartbeat_timer_;
    boost::asio::ip::tcp::endpoint m_ep;
};

// -----------------------------------------------------------------
// -----------------------------------------------------------------

async_task<int> mainflowWA(client& c1, client& c2, client& c3)
{
    print("mainflowWA: begin\n");

    for (int i = 0; i < 3; i++)
    {
        print("mainflowWA: async_operation sc1 = c1.start_connecting();\n");
        async_operation sc1 = c1.start_connecting();
        print("mainflowWA: async_operation sc2 = c2.start_connecting();\n");
        async_operation sc2 = c2.start_connecting();
        print("mainflowWA: async_operation sc3 = c3.start_connecting();\n");
        async_operation sc3 = c3.start_connecting();

        wait_all_awaitable<int> wac( { &sc1, &sc2, &sc3 } );
        print("mainflowWA: co_await wac;\n");
        co_await wac;

        print("mainflowWA: std::this_thread::sleep_for(std::chrono::seconds(2))\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        print("mainflow: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

        print("mainflowWA: async_operation sr1 = c1.start_reading();\n");
        async_operation sr1 = c1.start_reading();
        print("mainflowWA: async_operation sr2 = c2.start_reading();\n");
        async_operation sr2 = c2.start_reading();
        print("mainflowWA: async_operation sr3 = c3.start_reading();\n");
        async_operation sr3 = c3.start_reading();

        wait_all_awaitable<int> war( { &sr1, &sr2, &sr3 } );
        print("mainflowWA: co_await war;\n");
        co_await war;

        print("mainflow: !!!!!!!!!!!!!!!!!!!!\n");

        print("mainflow: c1.stop();\n");
        c1.stop();
        print("mainflow: c2.stop();\n");
        c2.stop();
        print("mainflow: c3.stop();\n");
        c3.stop();

        print("mainflowWA: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print("mainflowWA: std::this_thread::sleep_for(std::chrono::seconds(2))\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    print("mainflowWA: co_return 0;\n");
    co_return 0;
}

async_task<int> mainflowWA(std::initializer_list<client*> clients)
{
    async_operation asyncsc[3];
    async_operation asyncsr[3];

    int sz = clients.size();

    print("mainflow: begin\n");
    for (int i = 0; i < 3; i++)
    {
        int j;

        j = 0;
        for (client* cl : clients) {
            print("mainflow: asyncsc[%d] = cl->start_connecting();\n", j);
            asyncsc[j++] = cl->start_connecting();
        }
        wait_all_awaitable<int> wac(asyncsc, 3);
        print("mainflowWA: co_await wac;\n");
        co_await wac;

        print("mainflowWA: std::this_thread::sleep_for(std::chrono::seconds(2))\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        print("mainflow: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

        j = 0;
        for (client* cl : clients) {
            print("mainflow: asyncsr[%d] = cl->start_reading();\n", j);
            asyncsr[j++] = cl->start_reading();
        }
        wait_all_awaitable<int> war(asyncsr, 3);
        print("mainflowWA: co_await war;\n");
        co_await war;

        print("mainflow: !!!!!!!!!!!!!!!!!!!!\n");

        for (client* cl : clients) {
            print("mainflow: cl->stop();\n");
            cl->stop();
        }

        print("mainflow: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print("mainflowWA: std::this_thread::sleep_for(std::chrono::seconds(2))\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    print("mainflow: co_return 0;\n");
    co_return 0;
}

// -----------------------------------------------------------------

int main()
{
    boost::asio::io_context ioContext;

    print("main: client c(ioContext);\n");
    client c1(ioContext, ep);
    client c2(ioContext, ep);
    client c3(ioContext, ep);

#if 0
    print("main: async_task<int> si1 = mainflowWA(c1, c2, c3);\n");
    async_task<int> si1 = mainflowWA(c1, c2, c3);
#endif
#if 1
    print("main: async_task<int> si2 = mainflowWA( {&c1, &c2, &c3} )\n");
    async_task<int> si2 = mainflowWA({ &c1, &c2, &c3 });
#endif

    print("main: ioContext.run();\n");
    ioContext.run();

    print("main: return 0;\n");
    return 0;
}
