/**
 * @file hello-world-client.cpp
 * @brief
 * Source: https://stackoverflow.com/questions/14169971/how-to-create-tcp-client-by-libevent
 * With minor adaptations
 * 
 * Usqge examples:
 * hello-world-client.exe 9995 30 30 1
 * hello-world-client.exe 9995 30 1 1
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

void set_tcp_no_delay(evutil_socket_t fd)
{
    int one = 1;
	char *p = (char *)(&one);
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, const_cast<const char *>(p), sizeof(one));
}

void timeoutcb(evutil_socket_t fd, short what, void *arg)
{
    struct event_base *base = (struct event_base *)arg;
    printf("timeout\n");
    event_base_loopexit(base, NULL);
}

void readcb(struct bufferevent *bev, void *ctx)
{
    /* This callback is invoked when there is data to read on bev. */
    struct evbuffer *input = bufferevent_get_input(bev);
    //struct evbuffer *output = bufferevent_get_output(bev);

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

void eventcb(struct bufferevent *bev, short events, void *ptr)
{
    ++total_events;

    printf("eventcb: total_events = %zd\n", total_events);

    if (events & BEV_EVENT_CONNECTED)
    {
        evutil_socket_t fd = bufferevent_getfd(bev);
        set_tcp_no_delay(fd);
    } else if (events & BEV_EVENT_ERROR) {
        printf("NOT Connected\n");
    }
    else {
        printf("other\n");
    }
}

int main(int argc, char **argv)
{
    struct event_base  *base;
    struct bufferevent **bevs;
    struct sockaddr_in sin;
    struct event       *evtimeout;
    struct timeval     timeout;
    int i;

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
    timeout.tv_sec    = seconds;
    timeout.tv_usec   = 0;

#ifdef _WIN32
    WSADATA wsa_data;
    WSAStartup(0x0201, &wsa_data);
#endif

    base = event_base_new();
    if (!base) {
        puts("Couldn't open event base");
        return 1;
    }

    char* message = (char*)malloc(block_size);
    for (i = 0; i < block_size; ++i)
        message[i] = i % 128;
    memset(message,'A',block_size);

    evtimeout = evtimer_new(base, timeoutcb, base);
    evtimer_add(evtimeout, &timeout);

    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = htonl(0x7f000001); /* 127.0.0.1 */
    sin.sin_port = htons(port);

    bevs = (struct bufferevent **)malloc(session_count * sizeof(struct bufferevent *));

    for (i = 0; i < session_count; ++i)
    {
        struct bufferevent *bev = bufferevent_socket_new(base, -1, BEV_OPT_CLOSE_ON_FREE);

        bufferevent_setcb(bev, readcb, NULL, eventcb, NULL);
        bufferevent_enable(bev, EV_READ|EV_WRITE);
        evbuffer_add(bufferevent_get_output(bev), message, block_size);

        if (bufferevent_socket_connect(bev,(struct sockaddr *)&sin, sizeof(sin)) < 0)
        {
            /* Error starting connection */
            bufferevent_free(bev);
            puts("error connect");
            return -1;
        }
        bevs[i] = bev;
    }

    event_base_dispatch(base);

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
    return 0;
}