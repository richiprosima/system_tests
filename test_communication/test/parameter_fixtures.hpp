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

#ifndef __test_communication__parameter_fixtures__hpp__
#define __test_communication__parameter_fixtures__hpp__

#include <iostream>
#include <stdexcept>
#include <string>
#include "gtest/gtest.h"

#include <rclcpp/rclcpp.hpp>


void set_test_parameters(std::shared_ptr<rclcpp::parameter_client::SyncParametersClient> parameters_client)
{
  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foo.first", 8),
    rclcpp::parameter::ParameterVariant("foo.second", 42),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });

  // Check to see if they were set.
  for (auto & result : set_parameters_results) {
    ASSERT_TRUE(result.successful);
  }
}

void verify_test_parameters(std::shared_ptr<rclcpp::parameter_client::SyncParametersClient> parameters_client)
{
  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);
  for (auto & name : parameters_and_prefixes.names) {
    std::cout << "Parameter name: " << name << std::endl;
    EXPECT_TRUE(name == "foo" || name == "bar");
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }


  // Get a few of the parameters just set.
  for (auto & parameter : parameters_client->get_parameters({"foo", "bar", "baz"})) {
    std::cout << parameter.get_name() << parameter.to_string() << parameter.get_type_name() << std::endl;
    if (parameter.get_name() == "foo")
    {
      EXPECT_STREQ("2", parameter.to_string().c_str());
      EXPECT_STREQ("int", parameter.get_type_name().c_str());
    }
    else if (parameter.get_name() == "bar")
    {
      EXPECT_STREQ("hello", parameter.to_string().c_str());
      EXPECT_STREQ("string", parameter.get_type_name().c_str());
    }
    else if (parameter.get_name() == "baz")
    {
      EXPECT_STREQ("1.45", parameter.to_string().c_str());
      EXPECT_STREQ("double", parameter.get_type_name().c_str());
    }
    else
    {
      ASSERT_FALSE("you should never hit this");
    }
  }

  // Get a few non existant parameters
  for (auto & parameter : parameters_client->get_parameters({"not_foo", "not_baz"})) {
    EXPECT_STREQ("There should be no matches", parameter.get_name().c_str());
  }
}

#endif  // __test_communication__parameter_fixtures__hpp__
