/**
* @file addressfile.hpp
* @brief
*
* @author Johan Vanslembrouck
*/

#ifndef CPPCORO_ADDRESSFILE_HPP_INCLUDED
#define CPPCORO_ADDRESSFILE_HPP_INCLUDED

#include <cppcoro/net/socket.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>

void saveServerAddress(cppcoro::net::ip_endpoint serverAddress);
std::string readServerAddress();

#endif
