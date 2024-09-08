/**
 * @file multi-single_coroutine.cpp
 * @brief
 * Bosed upon multi-single-class.cpp, this code adds coroutine functionality.
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
 * using the multi interface to do a single download
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

using namespace corolib;

class CoCurl : public CommService
{
public:
    void init()
    {
        curl_global_init(CURL_GLOBAL_DEFAULT);

        /* init a multi stack */
        multi_handle = curl_multi_init();
    }

    void eventloop()
    {
        do {
            CURLMcode mc = curl_multi_perform(multi_handle, &still_running);

            if (!mc)
                /* wait for activity, timeout or "nothing" */
                mc = curl_multi_poll(multi_handle, NULL, 0, 1000, NULL);

            if (mc) {
                fprintf(stderr, "curl_multi_poll() failed, code %d.\n", (int)mc);
                break;
            }

        } while (still_running);
        // Call the event handler
        eventHandler();
    }

    void finalize_download()
    {
        curl_multi_remove_handle(multi_handle, http_handle);
        curl_easy_cleanup(http_handle);
    }

    void deinit()
    {
        curl_multi_cleanup(multi_handle);
        curl_global_cleanup();
    }

    // Coroutine paet

    async_operation<void> start_download(const char* url)
    {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        start_download_impl(index, url);
        return ret;
    }

    async_task<void> coroutine1()
    {
        co_await start_download("https://hanicka.net/a.txt");
        finalize_download();
        co_return;
    }

protected:
    void start_download_impl(const int idx, const char* url)
    {
        http_handle = curl_easy_init();

        /* set the options (I left out a few, you get the point anyway) */
        curl_easy_setopt(http_handle, CURLOPT_URL, url);

        /* add the individual transfers */
        curl_multi_add_handle(multi_handle, http_handle);

        eventHandler = [this, idx]()  {
            completionHandler_v(idx);
        };
    }

private:
    CURL* http_handle = nullptr;
    CURLM* multi_handle = nullptr;
    int still_running = 1; /* keep number of running handles */
    std::function<void(void)> eventHandler;
};

/*
 * Simply download an HTTP file.
 */
int main(void)
{
    CoCurl cocurl;
    cocurl.init();
    async_task<void> t = cocurl.coroutine1();
    cocurl.eventloop();
    t.wait();
    cocurl.deinit();
    return 0;
}
