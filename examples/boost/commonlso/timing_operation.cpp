/**
 * @file timing_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "timing_operation.h"

namespace corolib
{
    bool timing_operation_impl::try_start(async_operation_ls_base& operation) noexcept
    {
        print(PRI2, "%p: timing_operation_impl::try_start()\n", this);

        m_tmr.expires_after(std::chrono::milliseconds(m_time));

        m_tmr.async_wait(
            [this, &operation](const boost::system::error_code& error)
            {
                print(PRI2, "%p: CommCore::handle_timer(): entry\n", this);

                if (m_boost_context.m_stopped)
                {
                    print(PRI2, "%p: CommCore::handle_timer(): stopped\n");
                    return;
                }

                if (!error)
                {
                    operation.completed();
                }
                else
                {
                    print(PRI1, "%p: CommCore::handle_timer(): Error on timer: %s\n", this, error.message().c_str());
                    //stop();
                }
                print(PRI2, "%p: CommCore::handle_timer(): exit\n\n", this);
            });

        return true;
    }

    void timing_operation_impl::get_result(async_operation_ls_base&)
    {
        print(PRI2, "%p: timing_operation_impl::get_result()\n", this);
    }

}
