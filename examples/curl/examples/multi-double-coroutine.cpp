/**
 * @file multi-double-coroutine.cpp
 * @brief
 * Bosed upon multi-doubke-class.cpp, this code adds coroutine functionality.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
/***************************************************************************
 *                                  _   _ ____  _
 *  Project                     ___| | | |  _ \| |
 *                             / __| | | | |_) | |
 *                            | (__| |_| |  _ <| |___
 *                             \___|\___/|_| \_\_____|
 *
 * Copyright (C) Daniel Stenberg, <daniel@haxx.se>, et al.
 *
 * This software is licensed as described in the file COPYING, which
 * you should have received as part of this distribution. The terms
 * are also available at https://curl.se/docs/copyright.html.
 *
 * You may opt to use, copy, modify, merge, publish, distribute and/or sell
 * copies of the Software, and permit persons to whom the Software is
 * furnished to do so, under the terms of the COPYING file.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 * SPDX-License-Identifier: curl
 *
 ***************************************************************************/
/* <DESC>
 * multi interface code doing two parallel HTTP transfers
 * </DESC>
 */
#include <stdio.h>
#include <string.h>
#include <functional>

/* curl stuff */
#include <curl/curl.h>

/* corolib stuff */
#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

const char* URL1 = "https://hanicka.net/a.txt";
const char* URL2 = "https://hanicka.net/b.txt";

class CoCurl : public CommService
{
public:

    struct EventHandlerInfo
    {
        CURL* http_handle;
        std::function<void(CURLcode)> eventHandler;
    };

    CoCurl()
    {
        for (int i = 0; i < NROPERATIONS; ++i) {
            eventHandlerInfo[i].http_handle = nullptr;
        }
    }

    void init()
    {
        /* init a multi stack */
        multi_handle = curl_multi_init();
    }

    void eventloop()
    {
        while (still_running) {
            CURLMsg* msg;
            int queued;
            CURLMcode mc = curl_multi_perform(multi_handle, &still_running);

            if (still_running)
                /* wait for activity, timeout or "nothing" */
                mc = curl_multi_poll(multi_handle, NULL, 0, 1000, NULL);

            if (mc)
                break;

            do {
                msg = curl_multi_info_read(multi_handle, &queued);

                if (msg) {
                    if (msg->msg == CURLMSG_DONE) {
                        /* a transfer ended */
                        fprintf(stderr, "Transfer completed\n");
                        invoke_eventhandler(msg->easy_handle, msg->data.result);
                    }
                    else {
                        fprintf(stderr, "Got an unexpected message from curl: %i\n", msg->msg);
                    }
                }
            } while (msg);
        }
    }

    void remove_and_cleanup_handles()
    {
        for (int i = 0; i < NROPERATIONS; ++i) {
            if (eventHandlerInfo[i].http_handle) {
                curl_multi_remove_handle(multi_handle, eventHandlerInfo[i].http_handle);
                curl_easy_cleanup(eventHandlerInfo[i].http_handle);
            }
        }
    }

    void deinit()
    {
        curl_multi_cleanup(multi_handle);
    }

    // Coroutine part

    void invoke_eventhandler(CURL* h, CURLcode result)
    {
        for (int i = 0; i < NROPERATIONS; ++i) {
            if (eventHandlerInfo[i].http_handle == h) {
                eventHandlerInfo[i].eventHandler(result);
                break;
            }
        }
    }

    async_operation<CURLcode> start_download(const char* url)
    {
        int index = get_free_index();
        async_operation<CURLcode> ret{ this, index };
        start_download_impl(index, url);
        return ret;
    }

    async_task<void> coroutine1()
    {
        auto op1 = start_download(URL1);
        auto op2 = start_download(URL2);
        when_all wa(op1, op2);
        co_await wa;
        fprintf(stderr, "op1.get_result() = %d\n", op1.get_result());
        fprintf(stderr, "op2.get_result() = %d\n", op2.get_result());
        remove_and_cleanup_handles();
        co_return;
    }

protected:
    void start_download_impl(const int idx, const char* url)
    {
        eventHandlerInfo[idx].http_handle = curl_easy_init();

        /* set the options (I left out a few, you get the point anyway) */
        curl_easy_setopt(eventHandlerInfo[idx].http_handle, CURLOPT_URL, url);

        /* add the individual transfers */
        curl_multi_add_handle(multi_handle, eventHandlerInfo[idx].http_handle);

        eventHandlerInfo[idx].eventHandler = [this, idx](CURLcode result) {
            completionHandler(idx, result);
        };
    }

private:
    CURLM* multi_handle = nullptr;
    EventHandlerInfo eventHandlerInfo[NROPERATIONS];
    int still_running = 1; /* keep number of running handles */
};

void run_coroutine1(CoCurl& cocurl)
{
    async_task<void> t = cocurl.coroutine1();
    cocurl.eventloop();
    t.wait();
}

/*
 * Simply download two HTTP files!
 */
int main(void)
{
    CoCurl cocurl;
    cocurl.init();
    run_coroutine1(cocurl);
    cocurl.deinit();
    return 0;
}
