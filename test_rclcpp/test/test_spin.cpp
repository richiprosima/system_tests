// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <iostream>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  auto callback = [&promise](rclcpp::timer::TimerBase & timer) {
      promise.set_value(true);
      timer.cancel();
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(25), callback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_EQ(executor.spin_until_future_complete(future),
    rclcpp::executor::FutureReturnCode::SUCCESS);
  EXPECT_EQ(future.get(), true);
}

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete_timeout) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  auto callback = [&promise]() {
      promise.set_value(true);
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), callback);

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(25)),
    rclcpp::executor::FutureReturnCode::TIMEOUT);

  // If we wait a little longer, we should complete the future
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(50)),
    rclcpp::executor::FutureReturnCode::SUCCESS);

  EXPECT_EQ(future.get(), true);
}

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete_interrupted) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  auto callback = [&promise]() {
      promise.set_value(true);
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), callback);

  // Create a timer that will shut down rclcpp before
  auto shutdown_callback = []() {
      rclcpp::utilities::shutdown();
    };
  auto shutdown_timer = node->create_wall_timer(std::chrono::milliseconds(25), shutdown_callback);

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(50)),
    rclcpp::executor::FutureReturnCode::INTERRUPTED);
}

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), cancel) {
  auto node = rclcpp::Node::make_shared("cancel");
  rclcpp::executors::SingleThreadedExecutor executor;
  auto pub = node->create_publisher<test_rclcpp::msg::UInt32>("cancel", 10);
  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();

  auto subscription_callback = [](test_rclcpp::msg::UInt32::ConstSharedPtr)
    {
      fprintf(stderr, "Failure: subscription callback received before cancel\n");
      FAIL();
    };
  auto subscription = node->create_subscription<test_rclcpp::msg::UInt32>(
    "cancel", 10, subscription_callback);

  auto cancel_callback = [&executor, &pub, &msg]()
    {
      executor.cancel();
      // Try to publish after canceling. The callback should never trigger.
      pub->publish(msg);
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(5), cancel_callback);
  executor.add_node(node);
  executor.spin();
}

int main(int argc, char ** argv)
{
  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
