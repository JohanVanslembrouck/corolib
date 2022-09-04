/**
 *  Filename: p0510.cpp
 *  Description:
 *        Single-producer/single producer queue.
 *        The producer thread pushes elements to the queue,
 *        the consumer thread pops elements from the queue.
 *
 *        If the queue is empty, the consumer supsends
 *        and it will be resumed when the producer has
 *        pushed an element to the queue.
 *
 *        If the queue is full, the producer syspends
 *        and it will be resumed when the consumer has
 *        popped an element from the queue.
 *
 *        The push member function returns a
 *        TRxThreadQueueCor_push<T, Q_SIZE>
 *        coroutine type.
 *
 *        The pop member function returns a
 *        TRxThreadQueueCor_pop<T, Q_SIZE>
 *        coroutine type.
 *
 *        The coroutine handles of both coroutine types
 *        are stored in member functions of the queue type:
 *        TRxThreadQueueCor
 *
 *        This example shows how coroutines can be used to
 *        communicate between a producer and a consumer thread.
 *
 *        The print(PRI2, ...) calls in the implementation of
 *        the coroutine slow down the execution (compared to p0500.cpp).
 *        These statements should be commented out or deleted
 *        to speed up the application.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon:
 *
 */

#include <stdarg.h>
#include <time.h>
#include <limits.h>
#include <malloc.h>
#include <string.h>
#include <immintrin.h>

#include <atomic>
#include <cassert>
#include <iostream>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <stdio.h>

#include <experimental/resumable>

 //--------------------------------------------------------------

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
    return (uint64_t)(*ptr);
}

const int priority = 0x01;

void print(int pri, const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    if (priority & pri) {
        int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
            get_thread_number32((uint32_t)get_thread_id()) :
            get_thread_number64(get_thread_id());
        fprintf(stderr, "%02d: %s", threadid, msg);
    }
}

//--------------------------------------------------------------

template<typename T>
struct sync {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    sync(const sync& s) = delete;

    sync(sync&& s)
        : m_coroutine(s.m_coroutine) {
        print(PRI1, "sync::sync(sync&& s)\n");
        s.m_coroutine = nullptr;
    }

    ~sync() {
        print(PRI1, "sync::~sync()\n");
        //if (m_coroutine) m_coroutine.destroy();    // can throw exception when sync goes out of scope. FFS
    }

    sync& operator = (const sync&) = delete;

    sync& operator = (sync&& s) {
        print(PRI1, "sync::sync = (sync&& s)\n");
        m_coroutine = s.m_coroutine;
        s.m_coroutine = nullptr;
        return *this;
    }

    T get() {
        print(PRI1, "sync::get()\n");
        while (!m_coroutine.promise().set)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return m_coroutine.promise().value;
    }

    struct promise_type {

        friend struct sync;

        promise_type() : set(false) {
            print(PRI1, "sync::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI1, "sync::promise_type::~promise_type()\n");
        }

        auto return_value(T v) {
            print(PRI1, "sync::promise_type::return_value(T v)\n");
            this->value = v;
            this->set = true;
            if (this->m_awaiting)
                this->m_awaiting.resume();
            else
                print(PRI1, "sync::promise_type::return_value(T v): this->m_awaiting == nullptr\n");
        }

        auto get_return_object() {
            print(PRI1, "sync::promise_type::get_return_object()\n");
            return sync<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print(PRI1, "sync::promise_type::initial_suspend()\n");
            return std::experimental::suspend_never{};
        }

        auto final_suspend() {
            print(PRI1, "sync::promise_type::final_suspend()\n");
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI1, "sync::promise::promise_type()\n");
            std::exit(1);
        }

    private:
        T value;
        bool set;
        std::experimental::coroutine_handle<> m_awaiting;
    };

    sync(handle_type h)
        : m_coroutine(h) {
        print(PRI1, "sync::sync(handle_type h)\n");
    }

    handle_type m_coroutine;
};

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
class TRxThreadQueueCor_push;

template<typename T, unsigned long Q_SIZE>
class TRxThreadQueueCor_pop;

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
class TRxThreadQueueCor {
private:
    static const unsigned long Q_MASK = Q_SIZE - 1;

public:
    TRxThreadQueueCor()
        : head_(0)
        , tail_(0)
        , m_producer_waiting(false)
        , m_consumer_waiting(false)
        , push_not_ready(0)
        , resuming_push(0)
        , push_resumed(0)
        , pop_not_ready(0)
        , resuming_pop(0)
        , pop_resumed(0)
    {
        ptr_array_ = (T*) ::malloc(Q_SIZE * sizeof(T));
        assert(ptr_array_);
    }

    bool empty() {
        return (head_ == tail_);
    }

    bool full() {
        return (((head_ + 1) & Q_MASK) == tail_);
    }

    TRxThreadQueueCor_push<T, Q_SIZE>
        push(T messageIn);

    TRxThreadQueueCor_pop<T, Q_SIZE>
        pop();

    void printcontent();

    template<typename T, unsigned long Q_SIZE>
    friend class TRxThreadQueueCor_pop;

    template<typename T, unsigned long Q_SIZE>
    friend class TRxThreadQueueCor_push;

private:
    int m_activeConsumers;
    unsigned long head_;
    unsigned long tail_;

    std::experimental::coroutine_handle<> m_awaiting_prod;
    bool m_producer_waiting;

    std::experimental::coroutine_handle<> m_awaiting_cons;
    bool m_consumer_waiting;

    int push_not_ready;
    int resuming_push;
    int push_resumed;

    int pop_not_ready;
    int resuming_pop;
    int pop_resumed;

    T* ptr_array_;
    //T array_[Q_SIZE];
};

template<typename T, unsigned long Q_SIZE>
void TRxThreadQueueCor<T, Q_SIZE>::printcontent()
{
    fprintf(stderr, "push_not_ready = %d\n", push_not_ready);
    fprintf(stderr, "resuming_push  = %d\n", resuming_push);
    fprintf(stderr, "push_resumed   = %d\n", push_resumed);
    fprintf(stderr, "pop_not_ready  = %d\n", pop_not_ready);
    fprintf(stderr, "resuming_pop   = %d\n", resuming_pop);
    fprintf(stderr, "pop_resumed    = %d\n", pop_resumed);
    for (int i = 0; i < Q_SIZE; i++)
    {
        fprintf(stderr, "%d ", ptr_array_[i]);
        if ((i + 1) % 8 == 0)
            fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
}

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
class TRxThreadQueueCor_push
{
    static const unsigned long Q_MASK = Q_SIZE - 1;
public:
    TRxThreadQueueCor_push(TRxThreadQueueCor<T, Q_SIZE>& queue, T messageIn) :
        m_queue(queue),
        m_messageIn(messageIn),
        m_waiting(false)
    {
        print(PRI2, "TRxThreadQueueCor_push::TRxThreadQueueCor_push(queue)\n");
    }

    /**
     * @return true when the queue is not full and further pushes are possible.
     */
    bool await_ready()
    {
        bool full = m_queue.full();
        if (full) {
            print(PRI2, "TRxThreadQueueCor_push::await_ready(): return %d\n", !full);
            m_queue.push_not_ready++;
            m_waiting = true;
        }
        return !full;
    }

    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print(PRI2, "TRxThreadQueueCor_push::await_suspend(...)\n");
        m_queue.m_awaiting_prod = awaiting;
        m_queue.m_producer_waiting = true;
    }

    void await_resume() {
        if (m_waiting) {
            m_queue.push_resumed++;
            print(PRI2, "TRxThreadQueueCor_push::await_resume(): resumed\n");
            m_waiting = false;
        }
        bool empty = m_queue.empty();

        m_queue.ptr_array_[m_queue.head_] = m_messageIn;
        m_queue.head_ = (m_queue.head_ + 1) & Q_MASK;

        print(PRI2, "TRxThreadQueueCor_push::await_resume(): empty = %d : m_queue.m_consumer_waiting = %d\n", 
                    empty, m_queue.m_consumer_waiting);
        if (m_queue.m_consumer_waiting) {
            print(PRI2, "TRxThreadQueueCor_push::await_resume(): resuming consumer\n");
            m_queue.resuming_pop++;
            m_queue.m_consumer_waiting = false;
            m_queue.m_awaiting_cons.resume();
        }
    }

private:
    TRxThreadQueueCor<T, Q_SIZE>& m_queue;
    T m_messageIn;
    bool m_waiting;
};

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
class TRxThreadQueueCor_pop
{
    static const unsigned long Q_MASK = Q_SIZE - 1;
public:
    TRxThreadQueueCor_pop(TRxThreadQueueCor<T, Q_SIZE>& queue) :
        m_queue(queue),
        m_waiting(false)
    {
        print(PRI2, "TRxThreadQueueCor_pop::TRxThreadQueueCor_pop(queue)\n");
    }

    /**
     * @return true when the queue is not emptu and further pops are possible.
     */
    bool await_ready()
    {
        bool empty = m_queue.empty();
        if (empty) {
            print(PRI2, "TRxThreadQueueCor_pop::await_ready(): return %d\n", !empty);
            m_queue.pop_not_ready++;
            m_waiting = true;
        }
        return !empty;
    }

    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print(PRI2, "TRxThreadQueueCor_pop::await_suspend(...)\n");
        m_queue.m_awaiting_cons = awaiting;
        m_queue.m_consumer_waiting = true;
    }

    T await_resume() {
        if (m_waiting) {
            print(PRI2, "TRxThreadQueueCor_pop::await_resume(): resumed\n");
            m_queue.pop_resumed++;
            m_waiting = false;
        }
        bool full = m_queue.full();
        print(PRI2, "TRxThreadQueueCor_pop::await_resume(): full = %d: m_queue.m_producer_waiting = %d\n", 
            full, m_queue.m_producer_waiting);

        T messageOut = m_queue.ptr_array_[m_queue.tail_];
        m_queue.tail_ = (m_queue.tail_ + 1) & Q_MASK;

        print(PRI2, "TRxThreadQueueCor_pop::await_resume(): head_ = %d, tail_ = %d\n", m_queue.head_, m_queue.tail_);

        if (m_queue.m_producer_waiting) {
            print(PRI2, "TRxThreadQueueCor_pop::await_resume(): resuming producer\n");
            m_queue.resuming_push++;
            m_queue.m_producer_waiting = false;
            m_queue.m_awaiting_prod.resume();
        }

        return messageOut;
    }

private:
    TRxThreadQueueCor<T, Q_SIZE>& m_queue;
    bool m_waiting;
};

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
TRxThreadQueueCor_push<T, Q_SIZE>
TRxThreadQueueCor<T, Q_SIZE>::push(T messageIn)
{
    return TRxThreadQueueCor_push<T, Q_SIZE>(*this, messageIn);
}

template<typename T, unsigned long Q_SIZE>
TRxThreadQueueCor_pop<T, Q_SIZE>
TRxThreadQueueCor<T, Q_SIZE>::pop()
{
    return TRxThreadQueueCor_pop<T, Q_SIZE>(*this);
}

/*
 * ------------------------------------------------------------------------
 *    Tests
 * ------------------------------------------------------------------------
 */

static const auto CONSUMERS = 1;
static const auto PRODUCERS = 1;

template<typename T, unsigned long Q_SIZE>
sync<int> prod_coroutine(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q)
{
    print(PRI1, "prod_coroutine: begin\n");
    for (int i = 0; i < Q_SIZE * 1000 + Q_SIZE / 2; i++)
    {
        print(PRI2, "prod_coroutine: i = %d\n", i);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        TRxThreadQueueCor_push<T, Q_SIZE> ss = trxcor_q.push(i);
        co_await ss;
    }
    print(PRI1, "prod_coroutine: end\n");
    co_return 0;
}

template<typename T, unsigned long Q_SIZE>
void prod_function(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q)
{
    print(PRI1, "prod_function 1\n");
    sync<int> ii = prod_coroutine(trxcor_q);
    print(PRI1, "prod_function 2\n");
    int i = ii.get();
    print(PRI1, "prod_function 3: i = %d\n", i);
}

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
sync<int> cons_coroutine(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q)
{
    print(PRI1, "cons_coroutine: begin\n");
    T v = 0;
    for (int i = 0; i < Q_SIZE * 1000 + Q_SIZE / 2; i++)
    {
        print(PRI2, "cons_coroutine: i = %d, v = %d\n", i, v);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        TRxThreadQueueCor_pop<T, Q_SIZE> ss = trxcor_q.pop();
        v = co_await ss;
    }
    print(PRI1, "cons_coroutine: end\n");
    co_return 0;
}

template<typename T, unsigned long Q_SIZE>
void cons_function(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q)
{
    print(PRI1, "cons_function 1\n");
    sync<int> ii = cons_coroutine(trxcor_q);
    print(PRI1, "cons_function 2\n");
    int i = ii.get();
    print(PRI1, "cons_function 3: i = %d\n", i);
}

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
void run_test()
{
    TRxThreadQueueCor<T, Q_SIZE> trxcor_q;
    std::thread thr[PRODUCERS + CONSUMERS];

    print(PRI1, "run2 1: starting producers\n");
    for (int i = 0; i < PRODUCERS; ++i)
    {
        thr[i] = std::thread(
            [&trxcor_q]() -> void
            {
                prod_function(trxcor_q);
            }
        );
    }

    print(PRI1, "run2 2: starting consumers\n");
    for (int i = 0; i < CONSUMERS; ++i)
    {
        thr[PRODUCERS + i] = std::thread(
            [&trxcor_q]() -> void
            {
                cons_function(trxcor_q);
            }
        );
    }

    print(PRI1, "run2 3: waiting for join\n");
    // Wait for all threads to complete.
    for (auto i = 0; i < PRODUCERS + CONSUMERS; ++i)
        thr[i].join();

    print(PRI1, "run2 4: joined\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    trxcor_q.printcontent();
}

int main()
{
    run_test<uint16_t, 64>();
    return 0;
}

