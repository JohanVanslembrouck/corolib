/**
* @file cppcoro_wrapper.cpp
* @brief
*
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <functional>

#include <cppcoro/net/socket.hpp>

#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/CommService.h>
#include <corolib/print.h>

struct cppcoro_result
{
    cppcoro::detail::win32::dword_t m_errorCode{};
    cppcoro::detail::win32::dword_t m_numberOfBytesTransferred{};

    std::size_t get_result()
    {
        if (m_errorCode != 0)
        {
            throw std::system_error{
                static_cast<int>(m_errorCode),
                std::system_category()
            };
        }

        return m_numberOfBytesTransferred;
    }
};

struct cppcoro_wrapper : public corolib::CommService
{
    template<typename OPERATION>
    corolib::async_operation<cppcoro_result> start(OPERATION& op)
    {
        int index = get_free_index();
        corolib::async_operation<cppcoro_result> ret{ this, index };
        start_impl(op, index);
        return ret;
    }

    template<typename OPERATION>
    void start_impl(OPERATION& op, int idx)
    {
        if (op.try_start())
        {
            // asynchronous completion
            op.register_corolib_cb(
                [this, idx](cppcoro::detail::win32::dword_t errorCode,
                            cppcoro::detail::win32::dword_t numberOfBytesTransferred)
                {
                    cppcoro_result res{ errorCode, numberOfBytesTransferred };
                    completionHandler(idx, res);
                });
        }
        else
        {
            // synchronous completion
            cppcoro_result res{};
            op.get_results(res.m_errorCode, res.m_numberOfBytesTransferred);
            completionHandler(idx, res);
        }{}
    }


    corolib::async_task<void> accept(cppcoro::net::socket& s, cppcoro::net::socket& acceptingSocket) noexcept
    {
        cppcoro::net::socket_accept_operation sao = s.accept(acceptingSocket);
        co_await start(sao);
        co_return;
    }

    corolib::async_task<void> connect(cppcoro::net::socket& s, const cppcoro::net::ip_endpoint& remoteEndPoint) noexcept
    {
        cppcoro::net::socket_connect_operation sco = s.connect(remoteEndPoint);
        co_await start(sco);
        co_return;
    }

    corolib::async_task<void> disconnect(cppcoro::net::socket& s) noexcept
    {
        cppcoro::net::socket_disconnect_operation sdo = s.disconnect();
        co_await start(sdo);
        co_return;
    }

    corolib::async_task<std::size_t> recv(cppcoro::net::socket& s, void* buffer, std::size_t size) noexcept
    {
        cppcoro::net::socket_recv_operation sro = s.recv(buffer, size);
        cppcoro_result res = co_await start(sro);
        std::size_t bytesReceived = res.get_result();
        co_return bytesReceived;
    }

    corolib::async_task<std::size_t> send(cppcoro::net::socket& s, const void* buffer, std::size_t size) noexcept
    {
        cppcoro::net::socket_send_operation sso = s.send(buffer, size);
        cppcoro_result res = co_await start(sso);
        std::size_t bytesSent = res.get_result();
        co_return bytesSent;
    }

};

