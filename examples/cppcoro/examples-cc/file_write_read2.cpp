/**
* @file file_write_read2.cpp
* @brief
* Based upon TEST_CASE_FIXTURE(temp_dir_with_io_service_fixture, "read write file")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/file_tests.cpp
* Lambdas have been replaced by normal functions.
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/read_only_file.hpp>
#include <cppcoro/write_only_file.hpp>
#include <cppcoro/read_write_file.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/sync_wait.hpp>
#include <cppcoro/when_all.hpp>
#include <cppcoro/on_scope_exit.hpp>

#include <random>
#include <thread>
#include <cassert>
#include <string>

#include "io_service_fixture.hpp"
#include "temp_dir_fixture.hpp"

#include <ostream>

#include <iostream>
void CHECK(bool x)
{
    if (!x) std::cout << "error\n";
}

cppcoro::task<> write(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    std::printf(" starting write\n"); std::fflush(stdout);

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
        co_await f.write(chunk * sizeof(buffer), buffer, sizeof(buffer));
    }
}

cppcoro::task<> read(cppcoro::io_service& ioService, std::filesystem::path& filePath)
{
    std::printf(" starting read\n"); std::fflush(stdout);
 
    auto f = cppcoro::read_only_file::open(ioService, filePath);

    const auto fileSize = f.size();

    CHECK(fileSize == 10240);

    char buffer[20];

    for (std::uint64_t i = 0; i < fileSize;)
    {
        auto bytesRead = co_await f.read(i, buffer, 20);
        for (size_t j = 0; j < bytesRead; ++j, ++i)
        {
            CHECK(buffer[j] == ('a' + ((i % 1024) % 26)));
        }
    }
}

cppcoro::task<int> task1(cppcoro::io_service & ioService, std::filesystem::path& filePath)
{
    auto stopOnExit = cppcoro::on_scope_exit([&] { ioService.stop(); });
    co_await write(ioService, filePath);
    co_await read(ioService, filePath);
    co_return 0;
}

cppcoro::task<int> task2(cppcoro::io_service& ioService)
{
    ioService.process_events();
    co_return 0;
}

void mainflow()
{
    temp_dir_fixture temp_dir_fixture_;

	auto filePath = temp_dir_fixture_.temp_dir() / "foo";
    std::cout << "mainflow: filePath = " << filePath << "\n";

	cppcoro::io_service ioService;

    cppcoro::sync_wait(
        cppcoro::when_all(
            task1(ioService, filePath),
            task2(ioService)
		));
}

int main()
{
    std::cout << "main: entering\n";
    mainflow();
    std::cout << "main: leaving\n";
	return 0;
}
