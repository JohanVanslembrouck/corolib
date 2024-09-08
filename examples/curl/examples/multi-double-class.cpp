/**
 * @file multi-double-class.cpp
 * @brief
 * The code from multi-doubke.cpp has been placed in a class CoCurl and split into smaller functions.
 * This code will be used as a basis to implement the coroutine variant.
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

/* curl stuff */
#include <curl/curl.h>

class CoCurl
{
public:
    void init()
    {
        /* init a multi stack */
        multi_handle = curl_multi_init();
    }

    void start_download1()
    {
        http_handle = curl_easy_init();

        /* set options */
        //curl_easy_setopt(http_handle, CURLOPT_URL, "https://www.example.com/");
        curl_easy_setopt(http_handle, CURLOPT_URL, "https://hanicka.net/a.txt");

        /* add the individual transfers */
        curl_multi_add_handle(multi_handle, http_handle);
    }

    void start_download2()
    {
        http_handle2 = curl_easy_init();

        /* set options */
        //curl_easy_setopt(http_handle2, CURLOPT_URL, "http://localhost/");
        curl_easy_setopt(http_handle2, CURLOPT_URL, "https://hanicka.net/b.txt");

        /* add the individual transfers */
        curl_multi_add_handle(multi_handle, http_handle2);
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
                    }
                }
            } while (msg);
        }
    }

    void remove_and_cleanup_handle1()
    {
        curl_multi_remove_handle(multi_handle, http_handle);
        curl_easy_cleanup(http_handle);
    }

    void remove_and_cleanup_handle2()
    {
        curl_multi_remove_handle(multi_handle, http_handle2);
        curl_easy_cleanup(http_handle2);
    }

    void deinit()
    {
        curl_multi_cleanup(multi_handle);
    }

private:
    CURL* http_handle;
    CURL* http_handle2;
    CURLM* multi_handle;

    int still_running = 1; /* keep number of running handles */
};

/*
 * Simply download two HTTP files!
 */
int main(void)
{
    CoCurl cocurl;
    cocurl.init();
    cocurl.start_download1();
    cocurl.start_download2();
    cocurl.eventloop();
    cocurl.remove_and_cleanup_handle1();
    cocurl.remove_and_cleanup_handle2();
    cocurl.deinit();
    return 0;
}
