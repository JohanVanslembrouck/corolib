/**
 * @file operations-corolibthr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/async_operation.h"
#include "corolib/commservice.h"
#include "corolib/print.h"

#include "asyncthr.h"   // This is the only difference with operations-corolib.h

using namespace corolib;

class operationsCL : public CommService
{
public:
    async_operation<int> start_create(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index};
        async_create([this, index, i]() {
            print(PRI1, "async_create handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_create handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_open(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_open([this, index, i]() {
            print(PRI1, "async_open handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_open handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_write(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_write([this, index, i]() {
            print(PRI1, "async_write handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_write handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_read(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_read([this, index, i]() {
            print(PRI1, "async_read handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_read handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_close(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_close([this, index, i]() {
            print(PRI1, "async_close handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_close handler: end\n");
            });
        return ret;
    }

    async_operation<int> start_remove(int i)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_remove([this, index, i]() {
            print(PRI1, "async_remove handler: begin\n");
            completionHandler<int>(index, i);
            print(PRI1, "async_remove handler: end\n");
            });
        return ret;
    }
};
