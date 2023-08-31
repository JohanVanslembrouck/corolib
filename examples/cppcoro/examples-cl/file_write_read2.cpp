/**
* @file file_write_read2.cpp
* @brief
* Based upon ../examples-cc/file_write_read2.cpp
* 
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/read_only_file.hpp>
#include <cppcoro/write_only_file.hpp>
#include <cppcoro/read_write_file.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include <random>
#include <thread>
#include <cassert>
#include <string>

#include "cppcoro_wrapper.hpp"

#include "../examples-cc/io_service_fixture.hpp"
#include "../examples-cc/temp_dir_fixture.hpp"

#include <ostream>

using namespace corolib;

#include <iostream>
void CHECK(bool x)
{
    if (!x) std::cout << "error\n";
}

cppcoro_wrapper cc_wrapper;

async_task<void> write(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    std::printf(" starting write\n"); std::fflush(stdout);
    { // Without this extra scope, the application may sometimes abort. FFS.
        auto f = cppcoro::write_only_file::open(ioService, filePath);

        CHECK(f.size() == 0);

        char buffer[1024];
        char c = 'a';
        for (int i = 0; i < sizeof(buffer); ++i, c = (c == 'z' ? 'a' : c + 1))
        {
            buffer[i] = c;
        }

        for (int chunk = 0; chunk < 10; ++chunk)
        {
            // Original statement:
            // co_await f.write(chunk * sizeof(buffer), buffer, sizeof(buffer));
            co_await cc_wrapper.write(f, chunk * sizeof(buffer), buffer, sizeof(buffer));
        }
    }
    co_return;
}

async_task<void> read(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    std::printf(" starting read\n"); std::fflush(stdout);
    { // Without this extra scope, the application may sometimes abort. FFS.
        auto f = cppcoro::read_only_file::open(ioService, filePath);

        const auto fileSize = f.size();

        CHECK(fileSize == 10240);

        char buffer[20];

        for (std::uint64_t i = 0; i < fileSize;)
        {
            // Original statement:
            // auto bytesRead = co_await f.read(i, buffer, 20);
            auto bytesRead = co_await cc_wrapper.read(f, i, buffer, 20);
            for (size_t j = 0; j < bytesRead; ++j, ++i)
            {
                CHECK(buffer[j] == ('a' + ((i % 1024) % 26)));
            }
        }
    }
    co_return;
}

async_task<int> task1(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    co_await write(ioService, filePath);
    co_await read(ioService, filePath);
    co_return 0;
}

async_task<int> mainflow(cppcoro::io_service& ioService)
{
    temp_dir_fixture temp_dir_fixture_;

	auto filePath = temp_dir_fixture_.temp_dir() / "foo";
    std::cout << "mainflow: filePath = " << filePath << "\n";

    co_await task1(ioService, filePath);
    ioService.stop();
    co_return 0;
}

int main()
{
    std::cout << "main: entering\n";
    cppcoro::io_service ioService;
    mainflow(ioService);
    ioService.process_events();
    std::cout << "main: leaving\n";
	return 0;
}
