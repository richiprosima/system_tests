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

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

TEST(CLASSNAME(test_multiple_nodes, RMW_IMPLEMENTATION), spin) {
  auto node1 = rclcpp::Node::make_shared("test_multiple_nodes_1");
  auto node2 = rclcpp::Node::make_shared("test_multiple_nodes_2");
  size_t node1_messages_received = 0;
  size_t node2_messages_received = 0;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

  auto node2_callback =
    [&node2_messages_received](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++node2_messages_received;
      printf("  node2_callback() %lu with message data %u\n", node2_messages_received, msg->data);
    };

  auto node1_callback =
    [&node1_messages_received](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++node1_messages_received;
      printf("  node1_callback() %lu with message data %u\n", node1_messages_received, msg->data);
    };

  auto publisher1 = node1->create_publisher<test_rclcpp::msg::UInt32>("foo", qos_profile);
  auto subscriber1 = node2->create_subscription<test_rclcpp::msg::UInt32>("foo",
      qos_profile, node2_callback);

  auto publisher2 = node2->create_publisher<test_rclcpp::msg::UInt32>("bar", qos_profile);
  auto subscriber2 = node1->create_subscription<test_rclcpp::msg::UInt32>("bar",
      qos_profile, node1_callback);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  rclcpp::executors::SingleThreadedExecutor executor;

  for (size_t i = 0; i < 5; ++i) {
    msg->data = i;
    publisher1->publish(msg);
    publisher2->publish(msg);
    executor.spin_node_once(node1, std::chrono::milliseconds(0));
    executor.spin_node_once(node2, std::chrono::milliseconds(0));
  }

  ASSERT_EQ(node1_messages_received, 5);
  ASSERT_EQ(node2_messages_received, 5);

  //check that messages were received

}
