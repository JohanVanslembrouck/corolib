/**
 * @file p3000_async_api.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P3000_ASYNC_API_H_
#define _P3000_ASYNC_API_H_

#include <coroutine>

#include "p3000_async.h"

/**
 * @brief async_oper_base is used to place (pointers to) async operations of different types
 * in an array
 */
class async_oper_base
{
protected:
    std::coroutine_handle<> m_awaiting{ nullptr };
    bool m_ready{ false };
};

extern async_oper_base* async_oper_bases[32];

extern int start_index;

int get_free_index() {
    return start_index++;
}

template <typename TYPE>
class async_oper_baseT : public async_oper_base {
public:
    void set_result_and_resume(TYPE result = {}) {
        print(PRI2, "async_oper_baseT::set_result_and_resume(): begin\n");
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                print(PRI2, "async_oper_baseT::set_result_and_resume(): before m_awaiting.resume\n");
                tracker1_obj.nr_resumptions++;
                print(PRI2, "async_oper_baseT::set_result_and_resume(): tracker1_obj.nr_resumptions = %d\n", tracker1_obj.nr_resumptions);
                m_awaiting.resume();
                print(PRI2, "async_oper_baseT::set_result_and_resume(): after m_awaiting.resume\n");
            }
        }
        print(PRI2, "async_oper_baseT::set_result_and_resume(): end\n");
    }
protected:
    TYPE m_result{};
};

template<>
class async_oper_baseT<void> : public async_oper_base {
public:
    void set_result_and_resume() {
        print(PRI2, "async_oper_baseT::set_result_and_resume(): begin\n");
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                print(PRI2, "async_oper_baseT::set_result_and_resume(): before m_awaiting.resume\n");
                tracker1_obj.nr_resumptions++;
                print(PRI2, "async_oper_baseT::set_result_and_resume(): tracker1_obj.nr_resumptions\n", tracker1_obj.nr_resumptions);
                m_awaiting.resume();
                print(PRI2, "async_oper_baseT::set_result_and_resume(): after m_awaiting.resume\n");
            }
        }
        print(PRI2, "async_oper_baseT::set_result_and_resume(): end\n");
    }
};

template<typename TYPE>
void completionHandler(int index, TYPE val) {
    print(PRI2, "completionHandler(%d, val)\n", index);
    async_oper_base* op = async_oper_bases[index];
    if (op) {
        async_oper_baseT<TYPE>* opt = static_cast<async_oper_baseT<TYPE>*>(op);
        opt->set_result_and_resume(val);
    }
    else {
        print(PRI2, "completionHandler(%d): async_oper_bases[index] = %p\n", index, async_oper_bases[index]);
    }
}

void completionHandler(int index) {
    print(PRI2, "completionHandler(%d)\n", index);
    async_oper_base* op = async_oper_bases[index];
    if (op) {
        async_oper_baseT<void>* opt = static_cast<async_oper_baseT<void>*>(op);
        opt->set_result_and_resume();
    }
    else {
        print(PRI2, "completionHandler(%d): async_oper_bases[index] = %p\n", index, async_oper_bases[index]);
    }
}

#endif
