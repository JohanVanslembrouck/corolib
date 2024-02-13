/**
 * @file add_two_ints_client_async_co.cpp
 * @brief
 * Extended version of add_two_ints_client_async.cpp, including the use of coroutines.
 *
 * The original add_two_ints_client_async.cpp has first been extended by calling a function sum(a, b)
 * "recursively" (from the callback) until a certain limit has been reached.
 * 
 * When using coroutines, the recursive pattern is replaced with a while loop.
 * The callback function is used to resume the waiting coroutine.
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
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

/**
 https://github.com/ros2/example_interfaces/blob/humble/srv/AddTwoInts.srv
 https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Client.html
*/


#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

namespace demo_nodes_cpp
{
class ClientNodeCo : public rclcpp::Node, public CommService
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ClientNodeCo(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_client", options)
  {
    printf("Entering ClientNodeCo::ClientNodeCo\n");
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
	
    connect_to_service();
    
#if 0
    // Without coroutines
    sum(2, 3);
#else
    // With coroutines
    async_task<void> t = mainflowco();
#endif

    printf("Leaving ClientNodeCo::ClientNodeCo\n");
  }

  DEMO_NODES_CPP_PUBLIC
  void connect_to_service()
  {
      while (!client_->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return;
          }
          RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
  }

  DEMO_NODES_CPP_PUBLIC
  void sum(int a, int b)
  {
      printf("Entering sum\n");

      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = a;
      request->b = b;

      using ServiceResponseFuture =
          rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;

      auto response_received_callback = [this](ServiceResponseFuture future) {
          printf("Entering sum - callback\n");
          auto result = future.get();
          RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %" PRId64, result->sum);
          if (result->sum < 50)
              sum(result->sum, result->sum + 1);    // Calling sum again from the callback function
          else
              rclcpp::shutdown();
          printf("Leaving sum - callback\n");
      };

      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureAndRequestId future_result =
          client_->async_send_request(request, response_received_callback);

      printf("Leaving sum\n");
  }

  DEMO_NODES_CPP_PUBLIC
  async_task<void> mainflowco()
  {
      printf("Entering mainflowco\n");

      int a = 2;
      int b = 3;
      int res = 0;
      do
      {
          async_operation<int> ret = async_sum(a, b);
          res = co_await ret;
          printf("res = %d\n", res);
          a = res;
          b = a + 1;

      } while (res < 50);

      rclcpp::shutdown();

      printf("Leaving mainflowco\n");
  }

  DEMO_NODES_CPP_PUBLIC
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

      using ServiceResponseFuture =
          rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;

      auto response_received_callback = [this, idx] (ServiceResponseFuture future) {
          printf("Entering async_sum_impl - callback\n");
          auto result = future.get();
         
          async_operation_base* om_async_operation = m_async_operations[idx];
          async_operation<int>* om_async_operation_t =
              static_cast<async_operation<int>*>(om_async_operation);
          if (om_async_operation_t)
          {
              om_async_operation_t->set_result_and_complete(result->sum);
          }

          printf("Leaving async_sum_impl - callback\n");
      };

      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureAndRequestId future_result =
          client_->async_send_request(request, response_received_callback);

  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ClientNodeCo)
