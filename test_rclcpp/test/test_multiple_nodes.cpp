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

TEST(CLASSNAME(test_multiple_nodes, RMW_IMPLEMENTATION), spin_across_nodes_executors) {
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
  rclcpp::executors::SingleThreadedExecutor executor1;
  rclcpp::executors::SingleThreadedExecutor executor2;
  executor1.add_node(node1);
  executor2.add_node(node2);

  for (size_t i = 0; i < 5; ++i) {
    msg->data = i;
    publisher1->publish(msg);
    executor1.spin_some();
    executor2.spin_some();
    publisher2->publish(msg);
    executor1.spin_some();
    executor2.spin_some();
  }

  //check that messages were received
  EXPECT_EQ(node1_messages_received, 5);
  EXPECT_EQ(node2_messages_received, 5);
}

TEST(CLASSNAME(test_multiple_nodes, RMW_IMPLEMENTATION), spin_across_nodes) {
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
  executor.add_node(node1);
  executor.add_node(node2);

  for (size_t i = 0; i < 5; ++i) {
    msg->data = i;
    publisher1->publish(msg);
    executor.spin_some();
    publisher2->publish(msg);
    executor.spin_some();
  }

  //check that messages were received
  EXPECT_EQ(node1_messages_received, 5);
  EXPECT_EQ(node2_messages_received, 5);
}



// For comparison, this setup publishes the same topics within one node
TEST(CLASSNAME(test_multiple_nodes, RMW_IMPLEMENTATION), spin_one_node) {
  auto node1 = rclcpp::Node::make_shared("test_multiple_nodes_1");
  size_t callback1_messages_received = 0;
  size_t callback2_messages_received = 0;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

  auto callback2 =
    [&callback2_messages_received](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++callback2_messages_received;
      printf("  callback2() %lu with message data %u\n", callback2_messages_received, msg->data);
    };

  auto callback1 =
    [&callback1_messages_received](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++callback1_messages_received;
      printf("  callback1() %lu with message data %u\n", callback1_messages_received, msg->data);
    };

  auto publisher1 = node1->create_publisher<test_rclcpp::msg::UInt32>("foo", qos_profile);
  auto subscriber1 = node1->create_subscription<test_rclcpp::msg::UInt32>("foo",
      qos_profile, callback2);

  auto publisher2 = node1->create_publisher<test_rclcpp::msg::UInt32>("bar", qos_profile);
  auto subscriber2 = node1->create_subscription<test_rclcpp::msg::UInt32>("bar",
      qos_profile, callback1);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node1);

  for (size_t i = 0; i < 5; ++i) {
    msg->data = i;
    publisher1->publish(msg);
    executor.spin_some();
    publisher2->publish(msg);
    executor.spin_some();
  }

  //check that messages were received
  EXPECT_EQ(callback1_messages_received, 5);
  EXPECT_EQ(callback2_messages_received, 5);
}

// For comparison, this setup has two nodes, but does not publish across nodes
TEST(CLASSNAME(test_multiple_nodes, RMW_IMPLEMENTATION), spin_within_nodes) {
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
  auto subscriber1 = node1->create_subscription<test_rclcpp::msg::UInt32>("foo",
      qos_profile, node2_callback);

  auto publisher2 = node2->create_publisher<test_rclcpp::msg::UInt32>("bar", qos_profile);
  auto subscriber2 = node2->create_subscription<test_rclcpp::msg::UInt32>("bar",
      qos_profile, node1_callback);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);

  for (size_t i = 0; i < 5; ++i) {
    msg->data = i;
    publisher1->publish(msg);
    executor.spin_some();
    publisher2->publish(msg);
    executor.spin_some();
  }

  //check that messages were received
  EXPECT_EQ(node1_messages_received, 5);
  EXPECT_EQ(node2_messages_received, 5);
}
