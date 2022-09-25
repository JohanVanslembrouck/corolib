/**
 *  Filename: p0520.cpp
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
 *        In contrast to p510.cpp, consumer and producer
 *        run on the same thread.
 *
 *        The print(PRI2, ...) calls in the implementation of
 *        the coroutine slow down the execution (compared to p0500.cpp).
 *        These statements should be commented out or deleted
 *        to speed up the application.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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

#include <coroutine>

const int priority = 0x01;

#include "print.h"

//--------------------------------------------------------------

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

    std::coroutine_handle<> m_awaiting_prod;
    bool m_producer_waiting;

    std::coroutine_handle<> m_awaiting_cons;
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

    void await_suspend(std::coroutine_handle<> awaiting) {
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

    void await_suspend(std::coroutine_handle<> awaiting) {
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

template<typename T, unsigned long Q_SIZE>
oneway_task prod_coroutine(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q, int nrIterations)
{
    print(PRI1, "prod_coroutine: begin\n");
    for (int i = 0; i < nrIterations; i++)
    {
        print(PRI2, "prod_coroutine: i = %d\n", i);
        TRxThreadQueueCor_push<T, Q_SIZE> ss = trxcor_q.push(i);
        co_await ss;
    }
    print(PRI1, "prod_coroutine: end\n");
}

template<typename T, unsigned long Q_SIZE>
oneway_task cons_coroutine(TRxThreadQueueCor<T, Q_SIZE>& trxcor_q, int nrIterations)
{
    print(PRI1, "cons_coroutine: begin\n");
    for (int i = 0; i < nrIterations; i++)
    {
        print(PRI2, "cons_coroutine: i = %d\n", i);
        TRxThreadQueueCor_pop<T, Q_SIZE> ss = trxcor_q.pop();
        T v = co_await ss;
    }
    print(PRI1, "cons_coroutine: end\n");
}

//--------------------------------------------------------------

template<typename T, unsigned long Q_SIZE>
void run_test1()
{
    TRxThreadQueueCor<T, Q_SIZE> trxcor_q;

    print(PRI1, "run_test1(): prod_coroutine(trxcor_q, ...);\n");
    (void) prod_coroutine(trxcor_q, Q_SIZE * 1000 + Q_SIZE / 2);
    print(PRI1, "run_test1(): cons_coroutine(trxcor_q, ...);\n");
    (void) cons_coroutine(trxcor_q, Q_SIZE * 1000 + Q_SIZE / 2);

    print(PRI1, "run_test1(): trxcor_q.printcontent();\n");
    trxcor_q.printcontent();
}

template<typename T, unsigned long Q_SIZE>
void run_test2()
{
    TRxThreadQueueCor<T, Q_SIZE> trxcor_q;

    print(PRI1, "run_test2(): prod_coroutine(trxcor_q, ...);\n");
    (void) prod_coroutine(trxcor_q, Q_SIZE * 2000 + Q_SIZE / 2);
    print(PRI1, "run_test2(): cons_coroutine(trxcor_q, ...);\n");
    (void) cons_coroutine(trxcor_q, Q_SIZE * 1000 + Q_SIZE / 2);

    print(PRI1, "run_test2(): trxcor_q.printcontent();\n");
    trxcor_q.printcontent();
}

template<typename T, unsigned long Q_SIZE>
void run_test3()
{
    TRxThreadQueueCor<T, Q_SIZE> trxcor_q;

    print(PRI1, "run_test3(): cons_coroutine(trxcor_q, ...);\n");
    (void) cons_coroutine(trxcor_q, Q_SIZE * 2000 + Q_SIZE / 2);
    print(PRI1, "run_test3(): prod_coroutine(trxcor_q, ...);\n");
    (void) prod_coroutine(trxcor_q, Q_SIZE * 1000 + Q_SIZE / 2);

    print(PRI1, "run_test3(): trxcor_q.printcontent();\n");
    trxcor_q.printcontent();
}

int main()
{
    run_test1<uint16_t, 64>();
    run_test2<uint16_t, 64>();
    run_test3<uint16_t, 64>();
    return 0;
}
