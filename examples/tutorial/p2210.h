/**
 * @file p2210.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P2210_H_
#define _P2210_H_

#include <vector>

#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

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
        sort_operation_impl(Sorter* sorter, std::vector<int>& values);

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        Sorter* m_sorter;
        std::vector<int>& m_values;
    };

public:
    class sort_operation : public async_operation_ls<sort_operation>
    {
    public:
        sort_operation(Sorter* sorter, std::vector<int>& values)
            : m_impl(sorter, values)
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

    async_operation<void> start_sorting(std::vector<int>& values);
    sort_operation start_sorting_lso(std::vector<int>& values);

protected:
    void start_sorting_impl(int idx, std::vector<int>& values);

private:
    UseMode     m_useMode;
    EventQueueFunctionVoidVoid* m_eventQueue;
    EventQueueThrFunctionVoidVoid* m_eventQueueThr;
};

#endif
