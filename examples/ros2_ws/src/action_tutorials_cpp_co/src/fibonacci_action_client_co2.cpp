/**
 * @file fibonacci_action_client_co2.cpp
 * @brief
 * Second version with coroutines that propagates all results from the callback functions to 
 * the application level code.
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

class FibonacciActionClientCo2 : public rclcpp::Node, public CommService
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClientCo2(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client_co2", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
    
    corolib::set_priority(0x01);

    if (connect_to_server()) {
        mainflowco();
    }
  }

  enum class ActionStatus
  {
      UNKNOWN,

      GOAL_REJECTED,
      GOAL_ACCEPTED,

      FEEDBACK,

      RESULT_ABORTED,
      RESULT_CANCELLED,
      RESULT_SUCCEEDED
  };

  struct ActionResultInfo
  {
      ActionResultInfo(ActionStatus status = ActionStatus::UNKNOWN, int size = 0)
          : status(status)
          , sequence(size)
      {
      }

      ActionStatus status;
      std::vector<int32_t> sequence;
  };

  bool handle_result(ActionResultInfo& result)
  {
      bool finished = false;

      switch (result.status)
      {
      case ActionStatus::UNKNOWN:
          print(PRI1, "handle_result: ActionStatus::UNKNOWN\n");
          finished = true;
          break;
      case ActionStatus::GOAL_REJECTED:
          print(PRI1, "handle_result: ActionStatus::GOAL_REJECTED\n");
          finished = true;
          break;
      case ActionStatus::GOAL_ACCEPTED:
          print(PRI1, "handle_result: ActionStatus::GOAL_ACCEPTED\n");
          break;

      case ActionStatus::FEEDBACK:
          print(PRI1, "handle_result: ActionStatus::FEEDBACK\n");
          for (unsigned int i = 0; i < result.sequence.size(); i++)
              printf("%d ", result.sequence[i]);
          printf("\n");
          break;

      case ActionStatus::RESULT_ABORTED:
          print(PRI1, "handle_result: ActionStatus::RESULT_ABORTED\n");
          finished = true;
          break;
      case ActionStatus::RESULT_CANCELLED:
          print(PRI1, "handle_result: ActionStatus::RESULT_CANCELLED\n");
          finished = true;
          break;
      case ActionStatus::RESULT_SUCCEEDED:
          print(PRI1, "handle_result: ActionStatus::RESULT_SUCCEEDED\n");
          for (unsigned int i = 0; i < result.sequence.size(); i++)
              printf("%d ", result.sequence[i]);
          printf("\n");
          finished = true;
          break;
      }
      return finished;
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
      bool finished = false;

      async_operation<ActionResultInfo> op1 = send_goal_async(10);
      op1.auto_reset(true);
      do
      {
          ActionResultInfo result = co_await op1;
          finished = handle_result(result);
      } while (!finished);

      async_operation<ActionResultInfo> op2 = send_goal_async(15);
      op2.auto_reset(true);
      do
      {
          ActionResultInfo result = co_await op2;
          finished = handle_result(result);
      } while (!finished);

      async_operation<ActionResultInfo> op3 = send_goal_async(5);
      op3.auto_reset(true);
      do
      {
          ActionResultInfo result = co_await op3;
          finished = handle_result(result);
      } while (!finished);

      rclcpp::shutdown();
      co_return;
  }

  async_operation<ActionResultInfo> send_goal_async(int order)
  {
      int index = get_free_index();
      async_operation<ActionResultInfo> ret{ this, index };
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

              ActionResultInfo resultInfo(ActionStatus::GOAL_REJECTED);
              completionHandler<ActionResultInfo>(idx, resultInfo);
          }
          else {
              RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");

              ActionResultInfo resultInfo(ActionStatus::GOAL_ACCEPTED);
              completionHandler<ActionResultInfo>(idx, resultInfo);
          }
      };

      send_goal_options.feedback_callback =
          [this, idx](GoalHandleFibonacci::SharedPtr,
                      const std::shared_ptr<const Fibonacci::Feedback> feedback)
      {
          auto seq_size = feedback->partial_sequence.size();

          int i = 0;
          ActionResultInfo resultInfo(ActionStatus::FEEDBACK, seq_size);
          for (auto number : feedback->partial_sequence) {
              resultInfo.sequence[i] = number;
              i++;
          }
          completionHandler<ActionResultInfo>(idx, resultInfo);
      };

      send_goal_options.result_callback =
          [this, idx](const GoalHandleFibonacci::WrappedResult& result)
      {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              {
                  int i = 0;
                  auto seq_size = result.result->sequence.size();
                  ActionResultInfo resultInfo(ActionStatus::RESULT_SUCCEEDED, seq_size);
                  for (auto number : result.result->sequence) {
                      resultInfo.sequence[i] = number;
                      i++;
                  }
                  completionHandler<ActionResultInfo>(idx, resultInfo);
              }
              break;
          case rclcpp_action::ResultCode::ABORTED:
              {
                  RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                  ActionResultInfo resultInfo(ActionStatus::RESULT_ABORTED);
                  completionHandler<ActionResultInfo>(idx, resultInfo);
              }
              break;
          case rclcpp_action::ResultCode::CANCELED:
              {
                  RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                  ActionResultInfo resultInfo(ActionStatus::RESULT_CANCELLED);
                  completionHandler<ActionResultInfo>(idx, resultInfo);
              }
              break;
          case rclcpp_action::ResultCode::UNKNOWN:
              {
                  RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                  ActionResultInfo resultInfo(ActionStatus::UNKNOWN);
                  completionHandler<ActionResultInfo>(idx, resultInfo);
              }
              break;
          }
      };

      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // class FibonacciActionClientCo2

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClientCo2)
