/** 
 *  @file server.cpp
 *  @brief
 *  This example is based upon studies/corolab/p0700c.c.
 *  This example uses (where possible) lambdas instead of callback functions as in p0700c.c.
 * 
 *  This echo server is used for client1.cpp and client1.cpp clients.
 *
 *  This example does not use coroutines.
 *
 *  @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio.hpp>

#include <signal.h>
#include <boost/asio/signal_set.hpp>

#include <thread>
#include <string>

/// a server which is :
/// - ASYNCHRONOUS
/// - reads the message from the client
/// - sends this message back
/// - closes the connection

/// ===> scales, and if needed we can make it be served by multiple threads

#include "print.h"

//--------------------------------------------------------------

namespace
{
std::atomic_bool stop{false};

std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred)
{
    print("completionCondition(..., bytes_transferred = %ld)\n", bytes_transferred);
    if (!bytes_transferred)
    {
        return 1;
    }
    return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
}

/// client session
struct Clientx
{
    explicit Clientx(boost::asio::io_context& ioContext) :
        mReadBuffer{},
        mSendBuffer{},
        mSocket{ioContext}
    {
        print("Clientx::Clientx()\n");
    }

    ~Clientx()
    {
        print("Clientx::~Clientx()\n");
    }

    std::string mReadBuffer;
    std::string mSendBuffer;
    boost::asio::ip::tcp::socket mSocket;
};

using ClientSession = std::shared_ptr<Clientx>;

class Server
{
public:
    Server(
            boost::asio::io_context& ioContext,
            unsigned short service) :
        mIoContext{ioContext},
        mAcceptor{mIoContext, {boost::asio::ip::tcp::v4(), service}},
        mStop{false}
    {
        print("Server::Server(...)\n");
    }

    void start_accept()
    {
        print("Server::start_accept()\n");
        auto newClient = std::make_shared<Clientx>(mIoContext);

        mAcceptor.async_accept(
            newClient->mSocket,
            [this, newClient](const boost::system::error_code& ec)
            {
                print("Server::acceptHandler(...)\n");
                if (mStop)
                {
                    return;
                }
                if (ec)
                {
                    print("Server::acceptHandler(...): accept failed: %s\n", ec.message().c_str());
                }
                else
                {
                    start_read(newClient);
                }

                start_accept();
            });
    }

    void stop()
    {
        print("Server::stop()\n");
        mStop = true;
        mAcceptor.cancel();
    }

    void start_read(ClientSession client)
    {
        print("Server::start_read(ClientSession client)\n");
        boost::asio::async_read(
            client->mSocket,
            boost::asio::dynamic_buffer(client->mReadBuffer),
            std::bind(completionCondition, std::ref(client->mReadBuffer), std::placeholders::_1, std::placeholders::_2),
            std::bind(&Server::readHandler, this, client, std::placeholders::_1, std::placeholders::_2));
    }

    void readHandler(
            ClientSession client,
            const boost::system::error_code& ec,
            size_t bytes)
    {
        print("Server::readHandler(...)\n");
        if (mStop)
        {
            return;
        }
        if (ec)
        {
            print("Server::readHandler(...): read failed: %s\n", ec.message().c_str());
        }
        else
        {
            print("Server::readHandler(...): read %d bytes\n", bytes);
            client->mSendBuffer.resize(bytes);
            std::copy(client->mReadBuffer.cbegin(), client->mReadBuffer.cbegin() + bytes, client->mSendBuffer.begin());
            print("Server::readHandler(...): %s\n", client->mSendBuffer.c_str());

            std::this_thread::sleep_for(std::chrono::seconds(2)); // processing takes time

            start_write(client);
        }
    }

    void start_write(ClientSession client)
    {
        print("Server::start_write(ClientSession client)\n");

        boost::asio::async_write(
            client->mSocket,
            boost::asio::buffer(client->mSendBuffer),
            std::bind(&Server::writeHandler, this, client, std::placeholders::_1, std::placeholders::_2));
    }

    void writeHandler(
            ClientSession /*client*/,
            const boost::system::error_code& ec,
            size_t /*bytes*/)
    {
        print("Server::writeHandler(...)\n");
        if (mStop)
        {
            return;
        }
        if (ec)
        {
            print("Server::writeHandler(...): write failed: %s\n", ec.message().c_str());
        }
        /// we could choose to close the socket here, but since the session is going to end here
        /// the Client smart pointer is going to die and as such its socket member will get destructed
        ///        ==> socket closed
    }

private:
    boost::asio::io_context& mIoContext;
    boost::asio::ip::tcp::acceptor mAcceptor;
    std::atomic_bool mStop;
};


void runServer(
        Server& server,
        boost::asio::io_context& ioContext)
{
    print("runServer(...)\n");
    server.start_accept();

    ioContext.run();
}

void handlerSignals(
        const boost::system::error_code& /*erc*/,
        int signal)
{
    print("handlerSignals(...): signal %d occurred. Time to stop the application", signal);
    stop = true;
}

void asyncSignal(boost::asio::io_context& ioContext)
{
    print("asyncSignal(...)\n");
    boost::asio::signal_set signals{ioContext, SIGINT, SIGTERM};

    signals.async_wait(handlerSignals);
    ioContext.run();
}

} // namespace

int main()
{
    boost::asio::io_context ioContextSignal;
    boost::asio::io_context ioContextServer;

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

    Server server1{ioContextServer, 8242};
    std::thread t1{runServer, std::ref(server1), std::ref(ioContextServer)};
#if 0
    Server server2{ ioContextServer, 8342 };
    std::thread t2{ runServer, std::ref(server2), std::ref(ioContextServer) };

    Server server3{ ioContextServer, 8442 };
    std::thread t3{ runServer, std::ref(server3), std::ref(ioContextServer) };
#endif
    t.join(); /// wait on stop signal

    print("main(): server1.stop()\n");
    server1.stop();
    t1.join();
#if 0
    server2.stop();
    t2.join();

    server3.stop();
    t3.join();
#endif
    print("main(): return 0;\n");
    return 0;
}
