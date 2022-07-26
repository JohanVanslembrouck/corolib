/**
 * @file endpoints.h
 * @brief
 * Defines the endpoints (IP address and port) used by the clientserver and the server application.
 * In this setup all applications run locally.
 * The file should probably be replaced by a configuration file that is read at run time
 * to avoid having to recompile the application for every configuration.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _ENDPOINTS_H_
#define _ENDPOINTS_H_

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>

const char* ip_address_clientserver = "127.0.0.1";
const char* ip_address_server       = "127.0.0.1";

const int port_clientserver = 8242;
const int port_server       = 8342;

const boost::asio::ip::tcp::endpoint ep1{ boost::asio::ip::make_address(ip_address_clientserver), port_clientserver };
const boost::asio::ip::tcp::endpoint ep2{ boost::asio::ip::make_address(ip_address_server),		  port_server };

#endif
