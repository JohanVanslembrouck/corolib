/**
* @file cppcoro_wrapper.cpp
* @brief
* Wrapper for cppcoro "operation" classes.
*
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <iostream>
#include <functional>
#include <system_error>

#include <cppcoro/net/socket.hpp>

#include <cppcoro/read_only_file.hpp>
#include <cppcoro/readable_file.hpp>
#include <cppcoro/write_only_file.hpp>
#include <cppcoro/writable_file.hpp>

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
            corolib::print(corolib::PRI1, "cppcoro_result::get_result(): m_errorCode = %d: throw exception!\n", m_errorCode);
            throw std::system_error{ static_cast<int>(m_errorCode), std::system_category() };
        }

        return m_numberOfBytesTransferred;
    }
};

using recv_from_result_t = std::tuple<std::size_t, cppcoro::net::ip_endpoint>;

struct cppcoro_wrapper : public corolib::CommService
{
    template<typename OPERATION>
    corolib::async_operation<cppcoro_result> start(OPERATION& op) noexcept
    {
        int index = get_free_index();
        corolib::async_operation<cppcoro_result> ret{ this, index };
        start_impl(op, index);
        return ret;
    }

    template<typename OPERATION>
    void start_impl(OPERATION& op, int idx) noexcept
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
        }
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

    corolib::async_task<recv_from_result_t> recv_from(cppcoro::net::socket& s, void* buffer, std::size_t size)
    {
        cppcoro::net::socket_recv_from_operation srfo = s.recv_from(buffer, size);
        cppcoro_result res1 = co_await start(srfo);
        try {
            std::size_t bytesReceived = res1.get_result();
        }
        catch (...) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 1\n");
        }
        try {
            recv_from_result_t res = srfo.get_result();
            co_return res;
        }
        catch(...) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 2\n");
        }
        recv_from_result_t res{};
        co_return res;
    }

    corolib::async_task<std::size_t> send_to(
        cppcoro::net::socket& s,
        const cppcoro::net::ip_endpoint& remoteEndPoint,
        const void* buffer,
        std::size_t size)
    {
        cppcoro::net::socket_send_to_operation ssto = s.send_to(remoteEndPoint, buffer, size);
        cppcoro_result res = co_await start(ssto);
        try {
            std::size_t bytesSent = res.get_result();
            co_return bytesSent;
        }
        catch (...) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::send_to caught exception\n");
        }
        co_return 0;
    }

    // File operations

    corolib::async_task<std::size_t> read(
        cppcoro::read_only_file& file,
        std::uint64_t offset,
        void* buffer,
        std::size_t byteCount) noexcept
    {
        cppcoro::file_read_operation fro = file.read(offset, buffer, byteCount);
        cppcoro_result res = co_await start(fro);
        std::size_t bytesRead = res.get_result();
        co_return bytesRead;
    }

    corolib::async_task<std::size_t> write(
        cppcoro::write_only_file& file,
        std::uint64_t offset,
        void* buffer,
        std::size_t byteCount) noexcept
    {
        cppcoro::file_write_operation fwo = file.write(offset, buffer, byteCount);
        cppcoro_result res = co_await start(fwo);
        std::size_t bytesWritten = res.get_result();
        co_return bytesWritten;
    }

};

