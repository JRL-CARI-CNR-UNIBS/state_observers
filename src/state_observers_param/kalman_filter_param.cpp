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

#include "state_observers_param/kalman_filter_param.hpp"

namespace state_observer
{

void
KalmanFilterParam::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  StateObserverParam::initialize(node);

  fully_declare_parameter(
    "process_noise_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "Process noise covariance matrix (Q).");
  fully_declare_parameter(
    "measurement_noise_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "Measurement noise covariance matrix (R).");
  fully_declare_parameter(
    "initial_state_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "Initial state covariance matrix (P0).");

  std::vector<double> flat_Q, flat_R, flat_P0;
  if (!node_->get_parameter("process_noise_covariance", flat_Q)) {
    throw std::runtime_error("Failed to get process_noise_covariance parameter.");
  }
  if (flat_Q.size() != A_.rows() * A_.rows()) {
    throw std::runtime_error("Process noise covariance matrix has wrong size.");
  }

  if (!node_->get_parameter("measurement_noise_covariance", flat_R)) {
    throw std::runtime_error("Failed to get measurement_noise_covariance parameter.");
  }
  if (flat_R.size() != C_.rows() * C_.rows()) {
    throw std::runtime_error("Measurement noise covariance matrix has wrong size.");
  }

  if (!node_->get_parameter("initial_state_covariance", flat_P0)) {
    throw std::runtime_error("Failed to get initial_state_covariance parameter.");
  }

  if (flat_P0.size() != A_.rows() * A_.rows()) {
    throw std::runtime_error("Initial state covariance matrix has wrong size.");
  }

  Eigen::Map<Eigen::MatrixXd> Q_(flat_Q.data(), A_.rows(), A_.rows());
  Eigen::Map<Eigen::MatrixXd> R_(flat_R.data(), C_.rows(), C_.rows());
  Eigen::Map<Eigen::MatrixXd> P0_(flat_P0.data(), A_.rows(), A_.rows());
}


}  // namespace state_observer
