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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <test_rclcpp/msg/u_int32.hpp>

TEST(test_intra_process_within_one_node, nominal_usage) {
  rclcpp::init(0, nullptr);

  // use intra process = true
  auto node = rclcpp::Node::make_shared("test_intra_process", true);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 10;

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_intra_process", custom_qos_profile);

  unsigned long counter = 0;
  auto callback =
    [&counter](
    const test_rclcpp::msg::UInt32::SharedPtr msg,
    const rmw_message_info_t & message_info
    ) -> void
    {
      ++counter;
      printf("  callback() %lu with message data %u\n", counter, msg->data);
      ASSERT_EQ(counter, msg->data);
      ASSERT_TRUE(message_info.from_intra_process);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  {
    auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
      "test_intra_process", custom_qos_profile, callback, nullptr, true);

    // start condition
    ASSERT_EQ(0, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    ASSERT_EQ(0, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node);
    ASSERT_EQ(0, counter);

    msg->data = 1;
    publisher->publish(msg);
    ASSERT_EQ(0, counter);

    // wait for the first callback
    {
      size_t i = 0;
      while (counter == 0 && i < 2) {
        printf("spin_node_once() - callback (1) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node);
        if (counter != 0) {
          break;
        }
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
    ASSERT_EQ(1, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    ASSERT_EQ(1, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node);
    ASSERT_EQ(1, counter);

    msg->data = 2;
    publisher->publish(msg);
    msg->data = 3;
    publisher->publish(msg);
    msg->data = 4;
    publisher->publish(msg);
    msg->data = 5;
    publisher->publish(msg);
    ASSERT_EQ(1, counter);

    // while four messages have been published one callback should be triggered here
    {
      size_t i = 0;
      while (counter == 1 && i < 2) {
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        printf("spin_node_once(nonblocking) - callback (2) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node, std::chrono::milliseconds(0));
      }
    }
    ASSERT_EQ(2, counter);

    // check for next pending call
    {
      size_t i = 0;
      while (counter == 2 && i < 2) {
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        printf("spin_node_once(nonblocking) - callback (3) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node, std::chrono::milliseconds(0));
      }
    }
    ASSERT_EQ(3, counter);

    // check for all remaning calls
    printf("spin_node_some() - callbacks (4 and 5) expected\n");
    executor.spin_node_some(node);
    if (counter == 3 || counter == 4) {
      // give the executor thread time to process the event
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      printf("spin_node_some() - callback (%s) expected - trying again\n",
        counter == 3 ? "4 and 5" : "5");
      executor.spin_node_once(node, std::chrono::milliseconds(0));
    }
    ASSERT_EQ(5, counter);
  }
  // the subscriber goes out of scope and should be not receive any callbacks anymore

  msg->data = 6;
  publisher->publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // check that no further callbacks have been invoked
  printf("spin_node_some() - no callbacks expected\n");
  executor.spin_node_some(node);
  ASSERT_EQ(5, counter);
}

TEST(test_intra_process_two_nodes, nominal_usage) {
  rclcpp::init(0, nullptr);

  // use intra process = true
  auto node1 = rclcpp::Node::make_shared("test_intra_process1", true);
  auto node2 = rclcpp::Node::make_shared("test_intra_process2", true);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 10;

  // Publish from node 1
  auto publisher = node1->create_publisher<test_rclcpp::msg::UInt32>(
    "test_intra_process", custom_qos_profile);

  unsigned long counter = 0;
  auto callback =
    [&counter](
    const test_rclcpp::msg::UInt32::SharedPtr msg,
    const rmw_message_info_t & message_info
    ) -> void
    {
      ++counter;
      printf("  callback() %lu with message data %u\n", counter, msg->data);
      ASSERT_EQ(counter, msg->data);
      ASSERT_TRUE(message_info.from_intra_process);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  {
    // Subscribe in node 2
    auto subscriber = node2->create_subscription<test_rclcpp::msg::UInt32>(
      "test_intra_process", custom_qos_profile, callback, nullptr, true);

    // start condition
    ASSERT_EQ(0, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node1, std::chrono::milliseconds(0));
    executor.spin_node_once(node2, std::chrono::milliseconds(0));
    ASSERT_EQ(0, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node1);
    executor.spin_node_some(node2);
    ASSERT_EQ(0, counter);

    msg->data = 1;
    publisher->publish(msg);
    ASSERT_EQ(0, counter);

    // wait for the first callback
    {
      size_t i = 0;
      while (counter == 0 && i < 2) {
        printf("spin_node_once() - callback (1) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node1, std::chrono::milliseconds(0));
        executor.spin_node_once(node2, std::chrono::milliseconds(0));
        if (counter != 0) {
          break;
        }
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
    ASSERT_EQ(1, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node1, std::chrono::milliseconds(0));
    executor.spin_node_once(node2, std::chrono::milliseconds(0));
    ASSERT_EQ(1, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node1);
    executor.spin_node_some(node2);
    ASSERT_EQ(1, counter);

    msg->data = 2;
    publisher->publish(msg);
    msg->data = 3;
    publisher->publish(msg);
    msg->data = 4;
    publisher->publish(msg);
    msg->data = 5;
    publisher->publish(msg);
    ASSERT_EQ(1, counter);

    // while four messages have been published one callback should be triggered here
    {
      size_t i = 0;
      while (counter == 1 && i < 2) {
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        printf("spin_node_once(nonblocking) - callback (2) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node1, std::chrono::milliseconds(0));
        executor.spin_node_once(node2, std::chrono::milliseconds(0));
      }
    }
    ASSERT_EQ(2, counter);

    // check for next pending call
    {
      size_t i = 0;
      while (counter == 2 && i < 2) {
        // give the executor thread time to process the event
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        printf("spin_node_once(nonblocking) - callback (3) expected - try %zu/2\n", ++i);
        executor.spin_node_once(node1, std::chrono::milliseconds(0));
        executor.spin_node_once(node2, std::chrono::milliseconds(0));
      }
    }
    ASSERT_EQ(3, counter);

    // check for all remaning calls
    printf("spin_node_some() - callbacks (4 and 5) expected\n");
    executor.spin_node_some(node1);
    executor.spin_node_some(node2);
    if (counter == 3 || counter == 4) {
      // give the executor thread time to process the event
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      printf("spin_node_some() - callback (%s) expected - trying again\n",
        counter == 3 ? "4 and 5" : "5");
      executor.spin_node_once(node1, std::chrono::milliseconds(0));
      executor.spin_node_once(node2, std::chrono::milliseconds(0));
    }
    ASSERT_EQ(5, counter);
  }
  // the subscriber goes out of scope and should be not receive any callbacks anymore

  msg->data = 6;
  publisher->publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // check that no further callbacks have been invoked
  printf("spin_node_some() - no callbacks expected\n");
  executor.spin_node_some(node1);
  executor.spin_node_some(node2);
  ASSERT_EQ(5, counter);
}
