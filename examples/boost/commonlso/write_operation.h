/**
 * @file write_operation.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <corolib/async_operation.h>

#include "boost_context.h"

namespace corolib
{
    class write_operation_impl
    {
    public:
        write_operation_impl(boost_context& bc, const char* str, int size)
            : m_boost_context(bc)
            , m_str(str)
            , m_size(size)
        {

        }

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        boost_context& m_boost_context;
        const char* m_str;
        int m_size;
    };

    class write_operation : public async_operation_ls<write_operation>
    {
    public:
        write_operation(boost_context& bc, const char* str, int size)
            : m_impl(bc, str, size)
        {

        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        write_operation_impl m_impl;
    };

}
