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
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char ** argv)
{
  std::string node_name = "test_parameters";

  rclcpp::init(argc, argv);
  std::string test_server = node_name;
  if (argc >= 2) {
    std::string test_server = argv[1];
  }


  auto start = std::chrono::steady_clock::now();

  auto node = rclcpp::Node::make_shared(std::string(node_name));


  // TODO(esteve): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);


  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);

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
    if (!result.successful) {
      std::cerr << "Failed to set parameter: " << result.reason << std::endl;
      return 1;
    }
  }

  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);
  for (auto & name : parameters_and_prefixes.names) {
    std::cout << "Parameter name: " << name << std::endl;
    //TODO (tfoote) use an assert on the values
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    std::cout << "Parameter prefix: " << prefix << std::endl;
    //TODO (tfoote) use an assert on the values
  }


  // Get a few of the parameters just set.
  for (auto & parameter : parameters_client->get_parameters({"foo", "baz"})) {
    //TODO(tfoote) check the values here.
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): "
              << parameter.to_string() << std::endl;
    //TODO (tfoote) use an assert on the values
  }

  // Get a few non existant parameters
  for (auto & parameter : parameters_client->get_parameters({"not_foo", "not_baz"})) {
    std::cerr << "Should not get parameter name: " << parameter.get_name() << std::endl;
    std::cerr << "Should not get parameter value (" << parameter.get_type_name() << "): "
              << parameter.to_string() << std::endl;
    return 1;
  }

  auto async_parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node);

  // Set several differnet types of parameters.
  auto async_results = async_parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foo.first", 8),
    rclcpp::parameter::ParameterVariant("foo.second", 42),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });
  // Wait for the result.
  rclcpp::spin_until_future_complete(node, async_results);

  // Check to see if they were set.
  for (auto & result : async_results.get()) {
    if (!result.successful)
    {
      std::cerr << "Failed to set parameter: " << result.reason << std::endl;
      return 1;
    }
  }
  // List the details of a few parameters up to a namespace depth of 10.
  auto parameter_list_future = async_parameters_client->list_parameters({{"foo", "bar"}}, 10);
  auto parameter_list = rclcpp::spin_until_future_complete(node, parameter_list_future).get();
  for (auto & name : parameter_list.names) {
    std::cout << "Parameter name: " << name << std::endl;
  }
  for (auto & prefix : parameter_list.prefixes) {
    std::cout << "Parameter prefix: " << prefix << std::endl;
  }


  // Get a few of the parameters just set.
  auto parameters = async_parameters_client->get_parameters({{"foo", "baz"}});
  rclcpp::spin_until_future_complete(node, parameters);
  for (auto & parameter : parameters.get()) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): "
              << parameter.to_string() << std::endl;
    //TODO (tfoote) use an assert on the values
  }

  auto not_parameters = parameters_client->get_parameters({{"not_foo", "not_baz"}});
  rclcpp::spin_until_future_complete(node, parameters);
  for (auto & parameter : parameters.get()) {
    std::cerr << "Should not get parameter name: " << parameter.get_name() << std::endl;
    std::cerr << "Should not get parameter value (" << parameter.get_type_name() << "): "
              << parameter.to_string() << std::endl;
    return 1;
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "tested parameters for " << diff.count() << " seconds" << std::endl;

  return 0;
}
