/**
* @file file_write_read2.cpp
* @brief
* Based upon ../examples-cc/file_write_read2.cpp
* 
* @author Johan Vanslembrouck
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
    if (!x) print(PRI1, "error\n");
}

cppcoro_wrapper cc_wrapper;

async_task<void> write(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    print(PRI5, "write - entering\n");
    { // Without this extra scope, the application may sometimes abort. FFS.
        auto f = cppcoro::write_only_file::open(ioService, filePath);

        CHECK(f.size() == 0);

        write_only_file_wrapper wofw(f);

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
            co_await wofw.write(chunk * sizeof(buffer), buffer, sizeof(buffer));
            print(PRI5, "write - after co_await wofw.write\n");
        }
    }
    print(PRI5, "write - leaving\n");
    co_return;
}

async_task<void> read(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    print(PRI5, "read - entering\n");
    { // Without this extra scope, the application may sometimes abort. FFS.
        auto f = cppcoro::read_only_file::open(ioService, filePath);

        const auto fileSize = f.size();

        read_only_file_wrapper rofw(f);

        CHECK(fileSize == 10240);

        char buffer[20];

        for (std::uint64_t i = 0; i < fileSize;)
        {
            // Original statement:
            // auto bytesRead = co_await f.read(i, buffer, 20);
            auto bytesRead = co_await rofw.read(i, buffer, 20);
            print(PRI5, "read - after co_await rofw.read\n");
            for (size_t j = 0; j < bytesRead; ++j, ++i)
            {
                CHECK(buffer[j] == ('a' + ((i % 1024) % 26)));
            }
        }
    }
    print(PRI5, "read - leaving\n");
    co_return;
}

async_task<int> task1(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    print(PRI1, "task1 - entering\n");
    co_await write(ioService, filePath);
    co_await read(ioService, filePath);
    print(PRI1, "task1 - leaving\n");
    co_return 0;
}

async_task<int> mainflow(cppcoro::io_service& ioService)
{
    print(PRI1, "mainflow - entering\n");
    temp_dir_fixture temp_dir_fixture_;

	auto filePath = temp_dir_fixture_.temp_dir() / "foo";
    //print(PRI1, "mainflow - filePath = %s\n", filePath.c_str());
    co_await task1(ioService, filePath);
    print(PRI1, "mainflow - co_await task1\n");
    ioService.stop();
    print(PRI1, "mainflow - leaving\n");
    co_return 0;
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
                                // Use 0x11 to follow the flow in GreeterClient
    print(PRI1, "main - entering\n");
    cppcoro::io_service ioService;
    mainflow(ioService);
    ioService.process_events();
    print(PRI1, "main - leaving\n");
	return 0;
}
