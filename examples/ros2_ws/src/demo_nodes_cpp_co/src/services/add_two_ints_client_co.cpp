/**
 * @file add_two_ints_client_co.cpp
 * @brief
 * Extended version of add_two_ints_client.cpp, including the use of coroutines.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

struct ROSEvent
{
    ROSEvent(const std::function<void(example_interfaces::srv::AddTwoInts::Response::SharedPtr)>& eventHandler,
             std::future<rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedResponse>& fut,
             int64_t request_id)
        : m_eventHandler(std::move(eventHandler))
        , m_result(std::move(fut), request_id)
    {
        printf("Creating ROSEvent for request_id = %ld at address %p\n", m_result.request_id, static_cast<void*>(this));
    }

    ~ROSEvent()
    {
        printf("Deleting ROSEvent for request_id = %ld at address %p\n", m_result.request_id, static_cast<void*>(this));
    }

    std::function<void(example_interfaces::srv::AddTwoInts::Response::SharedPtr)> m_eventHandler;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::FutureAndRequestId m_result;
};

class Add_two_ints_Client : public CommService
{
public:

    Add_two_ints_Client(rclcpp::Node::SharedPtr node,
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client)
        : m_node(node)
        , m_client(client)
        , m_pROSEvent(nullptr)
    {

    }

    /**
     *  Based on the original implementation.
     */


    int sum(int a, int b)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        example_interfaces::srv::AddTwoInts::Response::SharedPtr result = send_request(request);

        if (result)
        {
            return result->sum;
        }
        else 
        {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for response. Exiting.");
        }
        return -1;
    }

    example_interfaces::srv::AddTwoInts::Response::SharedPtr
    send_request(example_interfaces::srv::AddTwoInts::Request::SharedPtr request)
    {
        //auto result = m_client->async_send_request(request);
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::FutureAndRequestId result = m_client->async_send_request(request);

        // Wait for the result.
        example_interfaces::srv::AddTwoInts::Response::SharedPtr res = handleEvent(result);
        return res;
    }

    example_interfaces::srv::AddTwoInts::Response::SharedPtr
    handleEvent(rclcpp::Client<example_interfaces::srv::AddTwoInts>::FutureAndRequestId& result)
    {
        if (rclcpp::spin_until_future_complete(m_node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return result.get();
        }
        else
        {
            return NULL;
        }
    }

    /**
     *  Using coroutines.
     */

    async_operation<int> async_sum(int a, int b)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        async_sum_impl(index, a, b);
        return ret;
    }

    void async_sum_impl(int idx, int a, int b)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        rclcpp::Client<example_interfaces::srv::AddTwoInts>::FutureAndRequestId result = m_client->async_send_request(request);

        auto eventHandler =
            [this, idx](example_interfaces::srv::AddTwoInts::Response::SharedPtr result)
        {
            int val = -1;
            if (result)
            {
                val = result->sum;
            }
            else
            {
                RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for response. Exiting.");
            }

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<int>* om_async_operation_t =
                static_cast<async_operation<int>*>(om_async_operation);
            if (om_async_operation_t)
            {
                om_async_operation_t->set_result_and_complete(val);
            }
        };

        m_pROSEvent = new ROSEvent(eventHandler, result.future, result.request_id);
    }

    void handleEvent()
    {
        ROSEvent* pRosEvent = nullptr;
        example_interfaces::srv::AddTwoInts::Response::SharedPtr ptr = nullptr;
        while (m_pROSEvent)
        {
            //if (pRosEvent)
            //    delete pRosEvent;
            pRosEvent = m_pROSEvent;
            m_pROSEvent = nullptr;

            if (rclcpp::spin_until_future_complete(m_node, pRosEvent->m_result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                ptr = pRosEvent->m_result.get();
            }
            pRosEvent->m_eventHandler(ptr);
            delete pRosEvent;
        }
    }

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr m_client;

    ROSEvent* m_pROSEvent;
};

void mainflow(Add_two_ints_Client& add_two_ints_Client)
{
    printf("Without coroutines\n");

    int res1 = add_two_ints_Client.sum(2, 3);
    printf("res1 = %d\n", res1);

    int res2 = add_two_ints_Client.sum(3, 5);
    printf("res2 = %d\n", res2);

    int res3 = add_two_ints_Client.sum(5, 8);
    printf("res3 = %d\n", res3);

    int res4 = add_two_ints_Client.sum(8, 13);
    printf("res4 = %d\n", res4);

    int res5 = add_two_ints_Client.sum(13, 21);
    printf("res5 = %d\n", res5);
}

async_task<void> mainflowco(Add_two_ints_Client& add_two_ints_Client)
{
    printf("With coroutines\n");

    async_operation<int> ret1 = add_two_ints_Client.async_sum(2, 3);
    int res1 = co_await ret1;
    printf("res1 = %d\n", res1);

    async_operation<int> ret2 = add_two_ints_Client.async_sum(3, 5);
    int res2 = co_await ret2;
    printf("res2 = %d\n", res2);

    async_operation<int> ret3 = add_two_ints_Client.async_sum(5, 8);
    int res3 = co_await ret3;
    printf("res3 = %d\n", res3);

    async_operation<int> ret4 = add_two_ints_Client.async_sum(8, 13);
    int res4 = co_await ret4;
    printf("res4 = %d\n", res4);

    async_operation<int> ret5 = add_two_ints_Client.async_sum(13, 21);
    int res5 = co_await ret5;
    printf("res5 = %d\n", res5);

    co_return;
}

int main(int argc, char ** argv)
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("add_two_ints_client_co");

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    Add_two_ints_Client add_two_ints_Client(node, client);
    mainflow(add_two_ints_Client);
    async_task<void> t = mainflowco(add_two_ints_Client);
    add_two_ints_Client.handleEvent();
    t.wait();

    rclcpp::shutdown();
    return 0;
}
