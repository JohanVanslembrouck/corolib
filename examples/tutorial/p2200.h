/**
 * @file 2200.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P2200_H_
#define _P2200_H_

#include <random>
#include <string>
#include <thread>
#include <vector>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#define USE_THREAD_POOL 1

#if USE_THREAD_POOL
#include <corolib/threadpool.h>
#endif

#include "use_mode.h"

#include "eventqueue.h"
#include "eventqueuethr.h"

using namespace corolib;

class Sorter : public CommService
{
private:
    class sort_operation_impl
    {
    public:
        sort_operation_impl(Sorter* sorter, std::vector<int>::iterator& begin, std::vector<int>::iterator& end);

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        Sorter* m_sorter;
        std::vector<int>::iterator& m_begin;
        std::vector<int>::iterator& m_end;
    };

public:
    class sort_operation : public async_operation_ls<sort_operation>
    {
    public:
        sort_operation(Sorter* sorter, std::vector<int>::iterator& begin, std::vector<int>::iterator& end)
            : m_impl(sorter, begin, end)
        {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        sort_operation_impl m_impl;
    };

public:
    Sorter(UseMode useMode = UseMode::USE_NONE,
        EventQueueFunctionVoidVoid* eventQueue = nullptr,
        EventQueueThrFunctionVoidVoid* eventQueueThr = nullptr)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
        , m_eventQueueThr(eventQueueThr)
    {
    }

    virtual ~Sorter() {}

    async_operation<void> start_sorting(auto begin, auto end);
    // gcc: if using 'auto', the application crashes during std::sort: iterators are not passed correctly:
    sort_operation start_sorting(Sorter* sorter, std::vector<int>::iterator& begin, std::vector<int>::iterator& end);

protected:
    void start_sorting_impl(int idx, auto begin, auto end);

private:
    UseMode     m_useMode;
    EventQueueFunctionVoidVoid* m_eventQueue;
    EventQueueThrFunctionVoidVoid* m_eventQueueThr;
#if USE_THREAD_POOL
    ThreadPool m_pool{ 8 };
#endif
};

// -----------------------------------------------------------------

async_task<void> sortCoroutine(Sorter& sorter, std::vector<int>& values);
async_task<void> sortCoroutine_lso(Sorter& sorter, std::vector<int>& values);

#endif
