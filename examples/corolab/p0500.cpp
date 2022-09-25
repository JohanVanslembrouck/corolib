/** 
 *  Filename: p0500.cpp
 *  Description:
 *        Single-producer/single producer queue.
 *        The producer thread pushes elements to the queue,
 *        the consumer thread pops elements from the queue.
 *        
 *        If the queue is empty, the consumer thread blocks
 *        and it will be awoken when the producer thread has
 *        pushed an element to the queue.
 *        Communication between both threads uses 
 *        std::condition_variable    cond_empty_;
 *
 *        If the queue is full, the producer thread blocks
 *        and will be awoken when the consumer thread has
 *        popped an element from the queue.
 *        Communication between both threads uses 
 *        std::condition_variable    cond_overflow_;
 *
 *        This example does not use coroutine features.
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

#define QUEUE_SIZE    64

#include "print0.h"

/*
 * ------------------------------------------------------------------------
 *
 * ------------------------------------------------------------------------
 */
template<class T, unsigned long Q_SIZE = QUEUE_SIZE>
class TRxThreadQueue {
private:
    static const unsigned long Q_MASK = Q_SIZE - 1;

public:
    TRxThreadQueue()
        : head_(0)
        , tail_(0)
        , resume_consumer(0)
        , resume_producer(0)
    {
        //ptr_array_ = (T * *)::memalign(getpagesize(), Q_SIZE * sizeof(void*));
        ptr_array_ = (T *) ::malloc(Q_SIZE * sizeof(T));
        assert(ptr_array_);
    }

    inline bool empty() {
        return (head_ == tail_);
    }

    inline bool full() {
        return (((head_ + 1) & Q_MASK) == tail_);
    }

    bool push(T messageIn) {
        std::unique_lock<std::mutex> lock(mtx_);

        while (full()) {
            //fprintf(stderr, "push(): full: head_ = %d, _tail = %d\n", head_, tail_);
            cond_overflow_.wait(lock, [this]() { return !full(); });
        }
        bool empty_ = empty();

        ptr_array_[head_] = messageIn;
        head_ = (head_ + 1) & Q_MASK;

        if (empty_) {
            resume_consumer++;
            cond_empty_.notify_one();
        }
        return true;
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mtx_);

        while (empty()) {
            //fprintf(stderr, "pop(): empty: head_ = %d, _tail = %d\n", head_, tail_);
            cond_empty_.wait(lock, [this]() { return !empty(); });
        }

        bool full_ = full();
    
        T messageOut = ptr_array_[tail_];
        tail_ = (tail_ + 1) & Q_MASK;

        if (full_) {
            resume_producer++;
            cond_overflow_.notify_one();
        }

        return messageOut;
    }

    void printcontent()
    {
        fprintf(stderr, "resume_consumer = %d\n", resume_consumer);
        fprintf(stderr, "resume_producer = %d\n", resume_producer);

        for (int i = 0; i < Q_SIZE; i++)
        {
            fprintf(stderr, "%d ", ptr_array_[i]);
            if ((i + 1) % 8 == 0)
                fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
    }

private:
    int activeConsumers_;

    unsigned long head_;
    unsigned long tail_;

    std::condition_variable    cond_empty_;
    std::condition_variable    cond_overflow_;
    std::mutex        mtx_;

    int resume_consumer;
    int resume_producer;

    T* ptr_array_;
};

/*
 * ------------------------------------------------------------------------
 *    Tests
 * ------------------------------------------------------------------------
 */
 
static const auto CONSUMERS = 1;
static const auto PRODUCERS = 1;

template<class Q>
struct Worker {
    Worker(Q* q, size_t id = 0)
        : q_(q),
        thr_id_(id)
    {}

    Q* q_;
    size_t thr_id_;
};

template<class Q>
struct Producer : public Worker<Q> {
    Producer(Q* q, size_t id)
        : Worker<Q>(q, id)
    {}

    void operator()()
    {
        for (int i = 0; i < QUEUE_SIZE * 1000 + QUEUE_SIZE / 2; i++) {
            Worker<Q>::q_->push(i);
        }
    }
};

template<class Q, typename T>
struct Consumer : public Worker<Q> {
    Consumer(Q* q, size_t id)
        : Worker<Q>(q, id)
    {}

    void operator()()
    {
        for (int i = 0; i < QUEUE_SIZE * 1000 + QUEUE_SIZE / 2; i++) {
            T v = Worker<Q>::q_->pop();
        }
    }
};

template<class Q, typename T>
void run_test(Q&& q)
{
    std::thread thr[PRODUCERS + CONSUMERS];

    // Run producers.
    for (auto i = 0; i < PRODUCERS; ++i)
        thr[i] = std::thread(Producer<Q>(&q, i));

    // Run consumers.
    for (auto i = 0; i < CONSUMERS; ++i)
        thr[PRODUCERS + i] = std::thread(Consumer<Q, T>(&q, i));

    // Wait for all threads completion.
    for (auto i = 0; i < PRODUCERS + CONSUMERS; ++i)
        thr[i].join();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    q.printcontent();
}

int main()
{
    TRxThreadQueue<uint16_t, QUEUE_SIZE> trx_q;
    run_test<TRxThreadQueue<uint16_t>, uint16_t>(std::move(trx_q));
    return 0;
}
