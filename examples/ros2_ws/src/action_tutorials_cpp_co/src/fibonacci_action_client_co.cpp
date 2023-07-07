/**
 * @file fibonacci_action_client_co.cpp
 * @brief
 * First version with coroutines with minimal changes to the original code in fibonacci_action_client.cpp
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

namespace action_tutorials_cpp
{
class FibonacciActionClientCo : public rclcpp::Node, public CommService
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClientCo(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client_co", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    if (connect_to_server()) {
        mainflowco();
    }
  }

  bool connect_to_server()
  {
      bool success = true;
      if (!this->client_ptr_->wait_for_action_server()) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();
          success = false;
      }
      return success;
  }

  async_task<void> mainflowco()
  {
      async_operation<bool> op1 = send_goal_async(10);
      bool success1 = co_await op1;
      printf("send_goal_async(10): success = %d\n", success1);

      async_operation<bool> op2 = send_goal_async(15);
      bool success2 = co_await op2;
      printf("send_goal_async(15): success = %d\n", success2);

      async_operation<bool> op3 = send_goal_async(5);
      bool success3 = co_await op3;
      printf("send_goal_async(5): success = %d\n", success3);

      rclcpp::shutdown();
      co_return;
  }

  async_operation<bool> send_goal_async(int order)
  {
      int index = get_free_index();
      async_operation<bool> ret{ this, index };
      async_send_goal_impl(index, order);
      return ret;
  }
  
  void async_send_goal_impl(int idx, int order)
  {
      auto goal_msg = Fibonacci::Goal();
      goal_msg.order = order;

      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

      send_goal_options.goal_response_callback =
          [this, idx](const GoalHandleFibonacci::SharedPtr& goal_handle)
      {
          if (!goal_handle) {
              RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
              completionHandler<bool>(idx, false);
          }
          else {
              RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
          }
      };

      send_goal_options.feedback_callback =
          [this](GoalHandleFibonacci::SharedPtr,
              const std::shared_ptr<const Fibonacci::Feedback> feedback)
      {
          std::stringstream ss;
          ss << "Next number in sequence received: ";
          for (auto number : feedback->partial_sequence) {
              ss << number << " ";
          }
          RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      };

      send_goal_options.result_callback =
          [this, idx](const GoalHandleFibonacci::WrappedResult& result)
      {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              break;
          case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
              completionHandler<bool>(idx, false);
              return;
          case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
              completionHandler<bool>(idx, false);
              return;
          default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code");
              completionHandler<bool>(idx, false);
              return;
          }

          std::stringstream ss;
          ss << "Result received: ";
          for (auto number : result.result->sequence) {
              ss << number << " ";
          }

          RCLCPP_INFO(this->get_logger(), ss.str().c_str());
          completionHandler<bool>(idx, true);
      };

      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // class FibonacciActionClientCo

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClientCo)
