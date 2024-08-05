// Copyright 2024 National Council of Research of Italy (CNR) - Intelligent Robotics Lab
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

#include "state_observers_param/state_observer_param.hpp"

namespace state_observer
{

void
StateObserverParam::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node_ = node;
  fully_declare_parameter("state_size", rclcpp::PARAMETER_INTEGER, "Size of the state vector.");
  fully_declare_parameter("input_size", rclcpp::PARAMETER_INTEGER, "Size of the input vector.");
  fully_declare_parameter("output_size", rclcpp::PARAMETER_INTEGER, "Size of the output vector.");
  fully_declare_parameter(
    "state_transition_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "State transition matrix (A).");
  fully_declare_parameter("input_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY, "Input matrix (B).");
  fully_declare_parameter("output_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY, "Output matrix (C).");
  fully_declare_parameter(
    "feedforward_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "Feedthrough matrix (D).");
  fully_declare_parameter("initial_state", rclcpp::PARAMETER_DOUBLE_ARRAY, "Initial state.");

  std::vector<double> flat_A, flat_B, flat_C, flat_D, initial_state;
  int state_size, input_size, output_size;

  node_->get_parameter("state_size", state_size);
  if (!node_->get_parameter("state_size", state_size)) {
    throw std::runtime_error("Failed to get state_size parameter.");
  }

  if (!node_->get_parameter("input_size", input_size)) {
    throw std::runtime_error("Failed to get input_size parameter.");
  }

  if (!node_->get_parameter("output_size", output_size)) {
    throw std::runtime_error("Failed to get output_size parameter.");
  }

  if (!node_->get_parameter("state_transition_matrix", flat_A)) {
    throw std::runtime_error("Failed to get state_transition_matrix parameter.");
  }
  if (flat_A.size() != state_size * state_size) {
    throw std::runtime_error("State transition matrix has wrong size.");
  }

  A_ = Eigen::Map<Eigen::MatrixXd>(flat_A.data(), state_size, state_size);
  if (node_->get_parameter("input_matrix", flat_B)) {
    if (flat_B.size() != state_size * input_size) {
      throw std::runtime_error("Input matrix has wrong size.");
    }
    B_ = Eigen::Map<Eigen::MatrixXd>(flat_B.data(), state_size, input_size);
  } else {
    B_ = Eigen::MatrixXd::Zero(state_size, input_size);
  }

  if (!node_->get_parameter("output_matrix", flat_C)) {
    throw std::runtime_error("Failed to get output_matrix parameter.");
  }
  if (flat_C.size() != output_size * state_size) {
    throw std::runtime_error("Output matrix has wrong size.");
  }
  C_ = Eigen::Map<Eigen::MatrixXd>(flat_C.data(), output_size, state_size);
  if (node_->get_parameter("feedforward_matrix", flat_D)) {
    if (flat_D.size() != output_size * input_size) {
      throw std::runtime_error("Feedthrough matrix has wrong size.");
    }
    D_ = Eigen::Map<Eigen::MatrixXd>(flat_D.data(), output_size, input_size);
  } else {
    D_ = Eigen::MatrixXd::Zero(output_size, input_size);
  }

  if (!node_->get_parameter("initial_state", initial_state)) {
    throw std::runtime_error("Failed to get initial_state parameter.");
  }
  if (initial_state.size() != state_size) {
    throw std::runtime_error("Initial state has wrong size.");
  }
  initial_state_ = Eigen::VectorXd::Map(initial_state.data(), initial_state.size());
}

void
StateObserverParam::fully_declare_parameter(
  const std::string & param_name,
  const rclcpp::ParameterType type,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.set__description(description);
  desc.type = type;
  node_->declare_parameter(param_name, type, desc);
}


}  // namespace state_observer
