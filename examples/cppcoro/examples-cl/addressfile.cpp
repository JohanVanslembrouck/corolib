/**
* @file addressfile.cpp
* @brief
*
* @author Johan Vanslembrouck
*/

#include <iostream>
#include <fstream>

#include <corolib/print.h>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

void saveServerAddress(ip_endpoint serverAddress)
{
    std::string str = serverAddress.to_string();
    print(PRI1, "serverAddress = %s\n", str.c_str());

    if (std::ofstream output{ "serverAddress.txt", std::ios::out })
    {
        output << str << std::endl;
    }
    else
    {
        print(PRI1, "File serverAddress.txt could not be opened\n");
        std::exit(EXIT_FAILURE);
    }
}

std::string readServerAddress()
{
    std::string serverAddressStr{};

    if (std::ifstream input{ "serverAddress.txt", std::ios::in })
    {
        while (input >> serverAddressStr)
        {
            print(PRI1, "serverAddress = %s\n", serverAddressStr.c_str());
        }
    }
    else
    {
        print(PRI1, "File serverAddress.txt could not be opened\n");
        std::exit(EXIT_FAILURE);
    }

    return serverAddressStr;
}
