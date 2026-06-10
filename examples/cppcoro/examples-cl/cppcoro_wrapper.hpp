/**
* @file cppcoro_wrapper.cpp
* @brief
* Wrapper for cppcoro "operation" classes.
*
* @author Johan Vanslembrouck
*/

#ifndef __CPPCORO_WRAPPER__
#define __CPPCORO_WRAPPER__

#include <iostream>
#include <functional>
#include <stdexcept>
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
#if CPPCORO_OS_WINNT
    cppcoro::detail::win32::dword_t m_errorCode{};
    cppcoro::detail::win32::dword_t m_numberOfBytesTransferred{};
#elif CPPCORO_OS_LINUX
    std::int32_t m_errorCode{};
    std::int32_t m_numberOfBytesTransferred{};
#endif
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

class cppcoro_wrapper : public corolib::CommService
{
protected:
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
            corolib::print(corolib::PRI2, "cppcoro_wrapper::start_impl: asynchronous completion\n");
            // asynchronous completion
#if CPPCORO_OS_WINNT
            // register_corolib_cb is defined in cppcoro/detail/win32_overlapped_operation.hpp
            op.register_corolib_cb(
                [this, idx](cppcoro::detail::win32::dword_t errorCode,
                            cppcoro::detail::win32::dword_t numberOfBytesTransferred)
                {
                    cppcoro_result res{ errorCode, numberOfBytesTransferred };
                    completionHandler(idx, res);
                });
#elif CPPCORO_OS_LINUX
            // register_corolib_cb is defined in cppcoro/detail/linux_async_operation.hpp
            op.register_corolib_cb(
                [this, idx](std::int32_t errorCode,
                            std::int32_t numberOfBytesTransferred)
                {
                    cppcoro_result res{ errorCode, numberOfBytesTransferred };
                    completionHandler(idx, res);
                });
#endif
        }
        else
        {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::start_impl: synchronous completion\n");
            // synchronous completion
            cppcoro_result res{};
            // get_results is defined in cppcoro/detail/win32_overlapped_operation.hpp
            //                    and in cppcoro/detail/linux_async_operation.hpp
            op.get_results(res.m_errorCode, res.m_numberOfBytesTransferred);
            completionHandler(idx, res);
        }
    }
};

class socket_wrapper : protected cppcoro_wrapper
{
public:
    socket_wrapper(cppcoro::net::socket s) :
        m_s(std::move(s))
    {
    }

    corolib::async_task<void> accept(cppcoro::net::socket& acceptingSocket) noexcept
    {
        cppcoro::net::socket_accept_operation sao = m_s.accept(acceptingSocket);
        cppcoro_result res = co_await start(sao);
#if CPPCORO_OS_WINNT
        cppcoro::detail::win32::socket_t handle = res.get_result();
        acceptingSocket.set_native_handle(handle);
#elif CPPCORO_OS_LINUX
        cppcoro::detail::linux::fd_t handle = res.get_result();
        acceptingSocket.set_native_handle(handle);
#endif
        co_return;
    }

    corolib::async_task<void> accept(cppcoro::net::socket& acceptingSocket, cppcoro::cancellation_token ct) noexcept
    {
        cppcoro::net::socket_accept_operation_cancellable sao = m_s.accept(acceptingSocket, ct);
        sao.initialize();
        cppcoro_result res = co_await start(sao);
#if CPPCORO_OS_WINNT
        cppcoro::detail::win32::socket_t handle = res.get_result();
        acceptingSocket.set_native_handle(handle);
#elif CPPCORO_OS_LINUX
        cppcoro::detail::linux::fd_t handle = res.get_result();
        acceptingSocket.set_native_handle(handle);
#endif
        co_return;
    }

    corolib::async_task<void> acceptOn(cppcoro::net::socket& listeningSocket) noexcept
    {
        cppcoro::net::socket_accept_operation sao = listeningSocket.accept(m_s);
        cppcoro_result res = co_await start(sao);
#if CPPCORO_OS_LINUX
        cppcoro::detail::linux::fd_t handle = res.get_result();
        m_s.set_native_handle(handle);
#endif
        co_return;
    }

    corolib::async_task<void> acceptOn(cppcoro::net::socket& listeningSocket, cppcoro::cancellation_token ct) noexcept
    {
        cppcoro::net::socket_accept_operation_cancellable sao = listeningSocket.accept(m_s, ct);
        sao.initialize();
        cppcoro_result res = co_await start(sao);
#if CPPCORO_OS_LINUX
        cppcoro::detail::linux::fd_t handle = res.get_result();
        m_s.set_native_handle(handle);
#endif
        co_return;
    }

    corolib::async_task<void> connect(const cppcoro::net::ip_endpoint& remoteEndPoint) noexcept
    {
        cppcoro::net::socket_connect_operation sco = m_s.connect(remoteEndPoint);
        cppcoro_result res = co_await start(sco);
        (void)res;
        co_return;
    }

    corolib::async_task<void> connect(const cppcoro::net::ip_endpoint& remoteEndPoint, cppcoro::cancellation_token ct) noexcept
    {
        cppcoro::net::socket_connect_operation_cancellable sco = m_s.connect(remoteEndPoint, ct);
        sco.initialize();
        cppcoro_result res = co_await start(sco);
        (void)res;
        co_return;
    }

    corolib::async_task<void> disconnect() noexcept
    {
        cppcoro::net::socket_disconnect_operation sdo = m_s.disconnect();
        cppcoro_result res = co_await start(sdo);
        (void)res;
        co_return;
    }

    corolib::async_task<std::size_t> recv(void* buffer, std::size_t size) noexcept
    {
        cppcoro::net::socket_recv_operation sro = m_s.recv(buffer, size);
        cppcoro_result res = co_await start(sro);
        std::size_t bytesReceived = res.get_result();
        co_return bytesReceived;
    }

    corolib::async_task<std::size_t> recv(void* buffer, std::size_t size, cppcoro::cancellation_token ct) noexcept
    {
        cppcoro::net::socket_recv_operation_cancellable sro = m_s.recv(buffer, size, ct);
        sro.initialize();
        cppcoro_result res = co_await start(sro);
        std::size_t bytesReceived = res.get_result();
        co_return bytesReceived;
    }

    corolib::async_task<std::size_t> send(const void* buffer, std::size_t size) noexcept
    {
        cppcoro::net::socket_send_operation sso = m_s.send(buffer, size);
        cppcoro_result res = co_await start(sso);
        std::size_t bytesSent = res.get_result();
        co_return bytesSent;
    }

    corolib::async_task<std::size_t> send(const void* buffer, std::size_t size, cppcoro::cancellation_token ct) noexcept
    {
        cppcoro::net::socket_send_operation_cancellable sso = m_s.send(buffer, size, ct);
        sso.initialize();
        cppcoro_result res = co_await start(sso);
        std::size_t bytesSent = res.get_result();
        co_return bytesSent;
    }

    corolib::async_task<recv_from_result_t> recv_from(void* buffer, std::size_t size)
    {
        cppcoro::net::socket_recv_from_operation srfo = m_s.recv_from(buffer, size);
        cppcoro_result res1 = co_await start(srfo);
        try {
            std::size_t bytesReceived = res1.get_result();
            (void)bytesReceived;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 1: %s\n", e.what());
            throw;
        }
        try {
            recv_from_result_t res = srfo.get_result();
            co_return res;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 2: %s\n", e.what());
            throw;
        }
        recv_from_result_t res{};
        co_return res;
    }

    corolib::async_task<recv_from_result_t> recv_from(void* buffer, std::size_t size, cppcoro::cancellation_token ct)
    {
        cppcoro::net::socket_recv_from_operation_cancellable srfo = m_s.recv_from(buffer, size, ct);
        srfo.initialize();
        cppcoro_result res1 = co_await start(srfo);
        try {
            std::size_t bytesReceived = res1.get_result();
            (void)bytesReceived;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 1: %s\n", e.what());
            throw;
        }
        try {
            recv_from_result_t res = srfo.get_result();
            co_return res;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::recv_from caught exception 2: %s\n", e.what());
            throw;
        }
        recv_from_result_t res{};
        co_return res;
    }

    corolib::async_task<std::size_t> send_to(
        const cppcoro::net::ip_endpoint& remoteEndPoint,
        const void* buffer,
        std::size_t size)
    {
        cppcoro::net::socket_send_to_operation ssto = m_s.send_to(remoteEndPoint, buffer, size);
        cppcoro_result res = co_await start(ssto);
        try {
            std::size_t bytesSent = res.get_result();
            co_return bytesSent;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::send_to caught exception: %s\n", e.what());
            throw;
        }
        co_return 0;
    }

    corolib::async_task<std::size_t> send_to(
        const cppcoro::net::ip_endpoint& remoteEndPoint,
        const void* buffer,
        std::size_t size,
        cppcoro::cancellation_token ct)
    {
        cppcoro::net::socket_send_to_operation_cancellable ssto = m_s.send_to(remoteEndPoint, buffer, size, ct);
        ssto.initialize();
        cppcoro_result res = co_await start(ssto);
        try {
            std::size_t bytesSent = res.get_result();
            co_return bytesSent;
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::send_to caught exception: %s\n", e.what());
            throw;
        }
        co_return 0;
    }

    void close_send()
    {
        try {
            m_s.close_send();
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::close_send caught exception: %s\n", e.what());
            corolib::print(corolib::PRI1, "                 m_s.native_handle() = %d\n", m_s.native_handle());
        }
    }

    void close_recv()
    {
        try {
            m_s.close_recv();
        }
        catch (const std::exception& e) {
            corolib::print(corolib::PRI1, "cppcoro_wrapper::close_recv caught exception: %s\n", e.what());
            corolib::print(corolib::PRI1, "                 m_s.native_handle() = %d\n", m_s.native_handle());
        }
    }

private:
    cppcoro::net::socket m_s;

};

class read_only_file_wrapper : protected cppcoro_wrapper
{
private:
    cppcoro::read_only_file m_file;

public:
    read_only_file_wrapper(cppcoro::read_only_file& file) :
        m_file(std::move(file))
    {
    }

    corolib::async_task<std::size_t> read(
        std::uint64_t offset,
        void* buffer,
        std::size_t byteCount) noexcept
    {
        cppcoro::file_read_operation fro = m_file.read(offset, buffer, byteCount);
        cppcoro_result res = co_await start(fro);
        std::size_t bytesRead = res.get_result();
        co_return bytesRead;
    }
};


class write_only_file_wrapper : protected cppcoro_wrapper
{
private:
    cppcoro::write_only_file m_file;

public:
    write_only_file_wrapper(cppcoro::write_only_file& file) :
        m_file(std::move(file))
    {
    }

    corolib::async_task<std::size_t> write(
        std::uint64_t offset,
        void* buffer,
        std::size_t byteCount) noexcept
    {
        cppcoro::file_write_operation fwo = m_file.write(offset, buffer, byteCount);
        cppcoro_result res = co_await start(fwo);
        std::size_t bytesWritten = res.get_result();
        co_return bytesWritten;
    }
};

#endif
