///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////

#include "socket_helpers.hpp"
#include <functional>
#include <cppcoro/net/ip_endpoint.hpp>

#include <cstring>
#include <cassert>
#if CPPCORO_OS_WINNT
# include <winsock2.h>
# include <ws2tcpip.h>
# include <mswsock.h>
# include <windows.h>
#elif CPPCORO_OS_LINUX
# include <sys/socket.h>
# include <netinet/in.h>
# include <netinet/tcp.h>
# include <netinet/udp.h>
#define SOCKADDR_IN sockaddr_in
#define SOCKADDR_IN6 sockaddr_in6
#endif


cppcoro::net::ip_endpoint
cppcoro::net::detail::sockaddr_to_ip_endpoint(const sockaddr& address) noexcept
{
	if (address.sa_family == AF_INET)
	{
		SOCKADDR_IN ipv4Address;
		std::memcpy(&ipv4Address, &address, sizeof(ipv4Address));

		std::uint8_t addressBytes[4];
		std::memcpy(addressBytes, &ipv4Address.sin_addr, 4);

		return ipv4_endpoint{
			ipv4_address{ addressBytes },
			ntohs(ipv4Address.sin_port)
		};
	}
	else
	{
		assert(address.sa_family == AF_INET6);

		SOCKADDR_IN6 ipv6Address;
		std::memcpy(&ipv6Address, &address, sizeof(ipv6Address));

		return ipv6_endpoint{
			ipv6_address{ ipv6Address.sin6_addr.s6_addr },
			ntohs(ipv6Address.sin6_port)
		};
	}
}

int cppcoro::net::detail::ip_endpoint_to_sockaddr(
	const ip_endpoint& endPoint,
	std::reference_wrapper<sockaddr_storage> address) noexcept
{
	if (endPoint.is_ipv4())
	{
		const auto& ipv4EndPoint = endPoint.to_ipv4();

		SOCKADDR_IN ipv4Address;
		ipv4Address.sin_family = AF_INET;
		std::memcpy(&ipv4Address.sin_addr, ipv4EndPoint.address().bytes(), 4);
		ipv4Address.sin_port = htons(ipv4EndPoint.port());
		std::memset(&ipv4Address.sin_zero, 0, sizeof(ipv4Address.sin_zero));

		std::memcpy(&address.get(), &ipv4Address, sizeof(ipv4Address));

		return sizeof(SOCKADDR_IN);
	}
	else
	{
		const auto& ipv6EndPoint = endPoint.to_ipv6();

		SOCKADDR_IN6 ipv6Address {0};
		ipv6Address.sin6_family = AF_INET6;
		std::memcpy(&ipv6Address.sin6_addr, ipv6EndPoint.address().bytes(), 16);
		ipv6Address.sin6_port = htons(ipv6EndPoint.port());
		ipv6Address.sin6_flowinfo = 0;

		std::memcpy(&address.get(), &ipv6Address, sizeof(ipv6Address));

		return sizeof(SOCKADDR_IN6);
	}
}
