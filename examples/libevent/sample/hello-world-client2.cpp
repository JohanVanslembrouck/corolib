/**
 * @file hello-world-client2.cpp
 * @brief
 * Source: https://stackoverflow.com/questions/14169971/how-to-create-tcp-client-by-libevent
 * Placed the code in a class to bridge the gap with the coroutine variont
 *
 * Usqge examples:
 * hello-world-client2.exe 9995 30 30 1
 * hello-world-client2.exe 9995 30 10 1
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <event2/event.h>
#include <event2/bufferevent.h>
#include <event2/buffer.h>

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

class tcpclient
{
public:

    static void set_tcp_no_delay(evutil_socket_t fd)
    {
        printf("set_tcp_no_delay\n");

        int one = 1;
        char* p = (char*)(&one);
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, const_cast<const char*>(p), sizeof(one));
    }

    static void timeoutcb(evutil_socket_t fd, short what, void* arg)
    {
        printf("timeoutcb\n");

        struct event_base* base = (struct event_base*)arg;
        event_base_loopexit(base, NULL);
    }

    static void readcb(struct bufferevent* bev, void* ctx)
    {
        printf("readcb\n");

        /* This callback is invoked when there is data to read on bev. */
        struct evbuffer* input = bufferevent_get_input(bev);
        //struct evbuffer* output = bufferevent_get_output(bev);

        size_t nr_read = 0;
        char* msg = evbuffer_readln(input, &nr_read, EVBUFFER_EOL_ANY);
        printf("msg = %s, nr_read = %zu\n", msg, nr_read);

        ++total_messages_read;
        total_bytes_read += evbuffer_get_length(input);

        printf("readcb: total_messages_read = %zd, total_bytes_read = %zd\n", total_messages_read, total_bytes_read);

        // Don't do this! Connection is already closed on Linux.
        /* Copy all the data from the input buffer to the output buffer. */
        //evbuffer_add_buffer(output, input);
    }

    static void eventcb(struct bufferevent* bev, short events, void* ptr)
    {
        ++total_events;

        printf("eventcb: total_events = %zd\n", total_events);

        if (events & BEV_EVENT_CONNECTED)
        {
            evutil_socket_t fd = bufferevent_getfd(bev);
            set_tcp_no_delay(fd);
        }
        else if (events & BEV_EVENT_ERROR) {
            printf("NOT Connected\n");
        }
        else {
            printf("other\n");
        }
    }

    int init(int port, int block_size, int session_count, int seconds)
    {
        timeout.tv_sec = seconds;
        timeout.tv_usec = 0;

        base = event_base_new();
        if (!base) {
            puts("Couldn't open event base");
            return 1;
        }

        message = (char*)malloc(block_size);
        for (i = 0; i < block_size; ++i)
            message[i] = i % 128;
        memset(message, 'A', block_size);

        evtimeout = evtimer_new(base, timeoutcb, base);
        evtimer_add(evtimeout, &timeout);

        memset(&sin, 0, sizeof(sin));
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = htonl(0x7f000001); /* 127.0.0.1 */
        sin.sin_port = htons(port);

        bevs = (struct bufferevent**)malloc(session_count * sizeof(struct bufferevent*));

        for (i = 0; i < session_count; ++i)
        {
            printf("i = % d\n", i);

            struct bufferevent* bev = bufferevent_socket_new(base, -1, BEV_OPT_CLOSE_ON_FREE);

            bufferevent_setcb(bev, readcb, NULL, eventcb, NULL);
            bufferevent_enable(bev, EV_READ | EV_WRITE);
            evbuffer_add(bufferevent_get_output(bev), message, block_size);

            if (bufferevent_socket_connect(bev, (struct sockaddr*)&sin, sizeof(sin)) < 0)
            {
                /* Error starting connection */
                bufferevent_free(bev);
                puts("error connect");
                return -1;
            }
            bevs[i] = bev;
        }
        return 0;
    }

    void deinit(int session_count)
    {
        for (i = 0; i < session_count; ++i) {
            bufferevent_free(bevs[i]);
        }

        free(bevs);
        event_free(evtimeout);
        event_base_free(base);
        free(message);

        printf("%zd total bytes read\n", total_bytes_read);
        printf("%zd total messages read\n", total_messages_read);
        printf("%.3f average messages size\n",
            (double)total_bytes_read / total_messages_read);
        printf("%.3f MiB/s throughtput\n",
            (double)total_bytes_read / (timeout.tv_sec * 1024 * 1024));
    }

public:
    struct event_base* base;
private:
    struct bufferevent** bevs;
    struct sockaddr_in sin;
    struct event* evtimeout;
    struct timeval     timeout;
    int i;
    char* message;
};

int main(int argc, char **argv)
{
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
    cl.init(port, block_size, session_count, seconds);;
    event_base_dispatch(cl.base);
    cl.deinit(session_count);
    return 0;
}