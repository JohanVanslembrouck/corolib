/**
 * @file hello-world-coroutine-client2.cpp
 * @brief
 * Coroutine variont of hello-world-client2.cpp
 *
 * Usqge examples:
 * hello-world-coroutine-client2.exe 9995 30 30 1
 * hello-world-coroutine-client2.exe 9995 30 10 1
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <event2/event.h>
#include <event2/bufferevent.h>
#include <event2/buffer.h>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#ifndef _WIN32
#include <netinet/in.h>
#include <netinet/tcp.h>
#ifdef _XOPEN_SOURCE_EXTENDED
#include <arpa/inet.h>
#endif
#include <sys/socket.h>
#endif

#include <string.h>
#include <stdlib.h>

int64_t total_bytes_read = 0;
int64_t total_messages_read = 0;
int64_t total_events = 0;

class tcpclient : public CommService
{
public:
    struct context_t
    {
        int idx;
        async_base* base;
    };

    static void set_tcp_no_delay(evutil_socket_t fd)
    {
        print(PRI1, "set_tcp_no_delay\n");

        int one = 1;
        char* p = (char*)(&one);
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, const_cast<const char*>(p), sizeof(one));
    }

    static void timeoutcb(evutil_socket_t fd, short what, void* arg)
    {
        print(PRI1, "timeoutcb\n");

        struct event_base* base = (struct event_base*)arg;
        print(PRI1, "timeout\n");
        event_base_loopexit(base, NULL);
    }

    static void readcb(struct bufferevent* bev, void* ctx)
    {
        print(PRI1, "readcb\n");

        /* This callback is invoked when there is data to read on bev. */
        struct evbuffer* input = bufferevent_get_input(bev);
        //struct evbuffer* output = bufferevent_get_output(bev);

        size_t nr_read = 0;
        char* msg = evbuffer_readln(input, &nr_read, EVBUFFER_EOL_ANY);
        print(PRI1, "msg = %s, nr_read = %zu\n", msg, nr_read);

        ++total_messages_read;
        total_bytes_read += evbuffer_get_length(input);

        print(PRI1, "readcb: total_messages_read = %zd, total_bytes_read = %zd\n", total_messages_read, total_bytes_read);

        // Don't do this! Connection is already closed on Linux.
        /* Copy all the data from the input buffer to the output buffer. */
        //evbuffer_add_buffer(output, input);

        context_t* ctxt = static_cast<context_t*>(ctx);
        int idx = ctxt->idx;
        async_base* om_async_base = ctxt->base;

        async_operation<std::string>* om_async_operation_t =
            static_cast<async_operation<std::string>*>(om_async_base);
        if (om_async_operation_t) {
            print(PRI1, "set_result: %d\n", idx);
            om_async_operation_t->set_result(msg);
        }
    }

    static void eventcb(struct bufferevent* bev, short events, void* ctx)
    {
        ++total_events;

        print(PRI1, "eventcb: total_events = %zd\n", total_events);

        if (events & BEV_EVENT_CONNECTED)
        {
            evutil_socket_t fd = bufferevent_getfd(bev);
            set_tcp_no_delay(fd);
        }
        else if (events & BEV_EVENT_ERROR) {
            print(PRI1, "NOT Connected\n");
        }
        else {
            context_t* ctxt = static_cast<context_t*>(ctx);
            int idx = ctxt->idx;
            async_base* om_async_base = ctxt->base;
            
            async_operation<std::string>* om_async_operation_t =
                static_cast<async_operation<std::string>*>(om_async_base);
            if (om_async_operation_t) {
                print(PRI1, "completing: %d\n", idx);
                om_async_operation_t->completed();
            }
        }
    }

    async_operation<std::string> start_operation(int& request, int block_size)
    {
        int index = get_free_index();
        async_operation<std::string> ret{ this, index };
        start_operation_impl(index, request, block_size);
        return ret;
    }

    int start_operation_impl(int idx, int request, int block_size)
    {
        print(PRI1, "start_operation_impl: idx = %d\n", idx);

        struct bufferevent* bev = bufferevent_socket_new(base, -1, BEV_OPT_CLOSE_ON_FREE);

        /*
        void bufferevent_setcb(struct bufferevent* bufev, 
                               bufferevent_data_cb readcb, 
                               bufferevent_data_cb writecb, 
                               bufferevent_event_cb eventcb, 
                               void* cbarg)
        */

        bufferevent_setcb(bev, readcb, NULL, eventcb, &ctxt[idx]);
        bufferevent_enable(bev, EV_READ | EV_WRITE);
        evbuffer_add(bufferevent_get_output(bev), message, block_size);

        if (bufferevent_socket_connect(bev, (struct sockaddr*)&sin, sizeof(sin)) < 0)
        {
            /* Error starting connection */
            bufferevent_free(bev);
            print(PRI1, "main() : error connect\n");
            return -1;
        }
        bevs[idx] = bev;
        return 0;
    }

    async_task<void> init(int port, int block_size, int session_count, int seconds)
    {
        timeout.tv_sec = seconds;
        timeout.tv_usec = 0;

        base = event_base_new();
        if (!base) {
            print(PRI1, "Couldn't open event base\n");
            co_return;
        }

        message = (char*) malloc(block_size);
        for (int i = 0; i < block_size; ++i)
            message[i] = i % 128;
        memset(message, 'A', block_size);

        evtimeout = evtimer_new(base, timeoutcb, base);
        evtimer_add(evtimeout, &timeout);

        memset(&sin, 0, sizeof(sin));
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = htonl(0x7f000001); /* 127.0.0.1 */
        sin.sin_port = htons(port);

        bevs = (struct bufferevent**) malloc(session_count * sizeof(struct bufferevent*));

        async_operation<std::string>* asyncsc = new async_operation<std::string>[session_count];
        ppasyncsc = new async_base * [session_count];
        ctxt = new context_t[session_count];
        for (int i = 0; i < session_count; ++i)
        {
            print(PRI1, "i = % d\n", i);
            ppasyncsc[i] = &asyncsc[i];
            ctxt[i].idx = i;
            ctxt[i].base = ppasyncsc[i];
            async_operation <std::string> op = start_operation(i, block_size);
            asyncsc[i] = std::move(op);
        }

        when_all wa(ppasyncsc, session_count);
        co_await wa;

        // print results
        for (int i = 0; i < session_count; ++i)
        {
            print(PRI1, "%s\n", asyncsc[i].get_result().c_str());
        }

        delete[] asyncsc;
        delete[] ppasyncsc;
        delete[] ctxt;
    }

    void deinit(int session_count)
    {
        for (int i = 0; i < session_count; ++i) {
            bufferevent_free(bevs[i]);
        }

        free(bevs);
        event_free(evtimeout);
        event_base_free(base);
        free(message);

        print(PRI1, "%zd total bytes read\n", total_bytes_read);
        print(PRI1, "%zd total messages read\n", total_messages_read);
        print(PRI1, "%.3f average messages size\n",
            (double)total_bytes_read / total_messages_read);
        print(PRI1, "%.3f MiB/s throughtput\n",
            (double)total_bytes_read / (timeout.tv_sec * 1024 * 1024));
    }

public:
    struct event_base* base;
    async_base** ppasyncsc;

private:
    struct bufferevent** bevs;
    struct sockaddr_in sin;
    struct event* evtimeout;
    struct timeval     timeout;
  
    context_t* ctxt;
    char* message;
};

int main(int argc, char **argv)
{
    set_priority(0x01);

    if (argc != 5)
    {
        fprintf(stderr, "Usage: client <port> <blocksize> ");
        fprintf(stderr, "<sessions> <time>\n");
        return 1;
    }

    int port          = atoi(argv[1]);
    int block_size    = atoi(argv[2]);
    int session_count = atoi(argv[3]);
    int seconds       = atoi(argv[4]);

#ifdef _WIN32
    WSADATA wsa_data;
    WSAStartup(0x0201, &wsa_data);
#endif

    tcpclient cl;
    async_task<void> t = cl.init(port, block_size, session_count, seconds);
    event_base_dispatch(cl.base);
    t.wait();
    cl.deinit(session_count);
    return 0;
}