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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <pluginlib/class_loader.hpp>

#include "state_observers_param/state_observer_param.hpp"
#include "state_observers_param/kalman_filter_param.hpp"

TEST(KalmanFilterParamTest, SuccessfulInitialization) {
  // Set expected parameter values
  int state_size = 3;
  int input_size = 2;
  int output_size = 1;

  std::vector<double> flat_A, flat_B, flat_C, flat_D, flat_initial_state;
  std::vector<double> flat_measurement_noise_covariance,
    flat_process_noise_covariance,
    flat_initial_state_covariance;

  Eigen::MatrixXd expected_A(state_size, state_size);
  expected_A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  Eigen::MatrixXd expected_B(state_size, input_size);
  expected_B << 10, 11, 12, 13, 14, 14;

  Eigen::MatrixXd expected_C(output_size, state_size);
  expected_C << 14, 15, 16;

  Eigen::MatrixXd expected_D(output_size, input_size);
  expected_D << 17, 18;

  Eigen::VectorXd expected_initial_state(state_size);
  expected_initial_state << 19, 20, 21;

  for (int i = 0; i < expected_A.size(); ++i) {
    flat_A.push_back(expected_A.data()[i]);
  }

  for (int i = 0; i < expected_B.size(); ++i) {
    flat_B.push_back(expected_B.data()[i]);
  }

  for (int i = 0; i < expected_C.size(); ++i) {
    flat_C.push_back(expected_C.data()[i]);
  }

  for (int i = 0; i < expected_D.size(); ++i) {
    flat_D.push_back(expected_D.data()[i]);
  }

  for (int i = 0; i < expected_initial_state.size(); ++i) {
    flat_initial_state.push_back(expected_initial_state.data()[i]);
  }

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters();


  node_options.append_parameter_override("state_size", 3);
  node_options.append_parameter_override("input_size", 2);
  node_options.append_parameter_override("output_size", 1);
  node_options.append_parameter_override("state_transition_matrix", flat_A);
  node_options.append_parameter_override("input_matrix", flat_B);
  node_options.append_parameter_override("output_matrix", flat_C);
  node_options.append_parameter_override("feedforward_matrix", flat_D);
  node_options.append_parameter_override("initial_state", flat_initial_state);

  Eigen::MatrixXd expected_process_noise_covariance(state_size, state_size);
  expected_process_noise_covariance << 10, 0.1, 0.1, 0.1, 10, 0.1, 0.1, 0.1, 10;

  for (int i = 0; i < expected_process_noise_covariance.size(); ++i) {
    flat_process_noise_covariance.push_back(expected_process_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_measurement_noise_covariance(output_size, output_size);
  expected_measurement_noise_covariance << 3;

  for (int i = 0; i < expected_measurement_noise_covariance.size(); ++i) {
    flat_measurement_noise_covariance.push_back(expected_measurement_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_initial_state_covariance(state_size, state_size);
  expected_initial_state_covariance << 2, 0.1, 0.1, 0.1, 2, 0.1, 0.1, 0.1, 2;

  for (int i = 0; i < expected_initial_state_covariance.size(); ++i) {
    flat_initial_state_covariance.push_back(expected_initial_state_covariance.data()[i]);
  }

  node_options.append_parameter_override("process_noise_covariance", flat_process_noise_covariance);
  node_options.append_parameter_override(
    "measurement_noise_covariance",
    flat_measurement_noise_covariance);
  node_options.append_parameter_override("initial_state_covariance", flat_initial_state_covariance);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "kalman_filter_param_node", node_options);

  state_observer::KalmanFilterParam observer_param;
  observer_param.initialize(node);

  ASSERT_TRUE(observer_param.get_A().isApprox(expected_A));
  ASSERT_TRUE(observer_param.get_B().isApprox(expected_B));
  ASSERT_TRUE(observer_param.get_C().isApprox(expected_C));
  ASSERT_TRUE(observer_param.get_D().isApprox(expected_D));
  ASSERT_TRUE(observer_param.get_initial_state().isApprox(expected_initial_state));
  ASSERT_TRUE(observer_param.get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(observer_param.get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(observer_param.get_P0().isApprox(expected_initial_state_covariance));
}

TEST(KalmanFilterParamLoaderTest, SuccessfulInitialization) {
  // Set expected parameter values
  int state_size = 3;
  int input_size = 2;
  int output_size = 1;

  std::vector<double> flat_A, flat_B, flat_C, flat_D, flat_initial_state;
  std::vector<double> flat_measurement_noise_covariance,
    flat_process_noise_covariance,
    flat_initial_state_covariance;

  Eigen::MatrixXd expected_A(state_size, state_size);
  expected_A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  Eigen::MatrixXd expected_B(state_size, input_size);
  expected_B << 10, 11, 12, 13, 14, 14;

  Eigen::MatrixXd expected_C(output_size, state_size);
  expected_C << 14, 15, 16;

  Eigen::MatrixXd expected_D(output_size, input_size);
  expected_D << 17, 18;

  Eigen::VectorXd expected_initial_state(state_size);
  expected_initial_state << 19, 20, 21;

  for (int i = 0; i < expected_A.size(); ++i) {
    flat_A.push_back(expected_A.data()[i]);
  }

  for (int i = 0; i < expected_B.size(); ++i) {
    flat_B.push_back(expected_B.data()[i]);
  }

  for (int i = 0; i < expected_C.size(); ++i) {
    flat_C.push_back(expected_C.data()[i]);
  }

  for (int i = 0; i < expected_D.size(); ++i) {
    flat_D.push_back(expected_D.data()[i]);
  }

  for (int i = 0; i < expected_initial_state.size(); ++i) {
    flat_initial_state.push_back(expected_initial_state.data()[i]);
  }

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters();


  node_options.append_parameter_override("state_size", 3);
  node_options.append_parameter_override("input_size", 2);
  node_options.append_parameter_override("output_size", 1);
  node_options.append_parameter_override("state_transition_matrix", flat_A);
  node_options.append_parameter_override("input_matrix", flat_B);
  node_options.append_parameter_override("output_matrix", flat_C);
  node_options.append_parameter_override("feedforward_matrix", flat_D);
  node_options.append_parameter_override("initial_state", flat_initial_state);

  Eigen::MatrixXd expected_process_noise_covariance(state_size, state_size);
  expected_process_noise_covariance << 10, 0.1, 0.1, 0.1, 10, 0.1, 0.1, 0.1, 10;

  for (int i = 0; i < expected_process_noise_covariance.size(); ++i) {
    flat_process_noise_covariance.push_back(expected_process_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_measurement_noise_covariance(output_size, output_size);
  expected_measurement_noise_covariance << 3;

  for (int i = 0; i < expected_measurement_noise_covariance.size(); ++i) {
    flat_measurement_noise_covariance.push_back(expected_measurement_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_initial_state_covariance(state_size, state_size);
  expected_initial_state_covariance << 2, 0.1, 0.1, 0.1, 2, 0.1, 0.1, 0.1, 2;

  for (int i = 0; i < expected_initial_state_covariance.size(); ++i) {
    flat_initial_state_covariance.push_back(expected_initial_state_covariance.data()[i]);
  }


  node_options.append_parameter_override("process_noise_covariance", flat_process_noise_covariance);
  node_options.append_parameter_override(
    "measurement_noise_covariance",
    flat_measurement_noise_covariance);
  node_options.append_parameter_override("initial_state_covariance", flat_initial_state_covariance);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "kalman_filter_param_node", node_options);

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserverParam>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserverParam>(
      "state_observers",
      "state_observer::StateObserverParam"));
  std::shared_ptr<state_observer::StateObserverParam> state_observer_param;

  try {
    state_observer_param =
      state_observer_loader->createSharedInstance(
      "state_observer::KalmanFilterParam");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  state_observer_param->initialize(node);
  ASSERT_TRUE(state_observer_param->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer_param->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer_param->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer_param->get_D().isApprox(expected_D));
  ASSERT_TRUE(state_observer_param->get_initial_state().isApprox(expected_initial_state));

  std::shared_ptr<state_observer::KalmanFilterParam> kalman_filter_param =
    std::dynamic_pointer_cast<state_observer::KalmanFilterParam>(state_observer_param);

  ASSERT_TRUE(kalman_filter_param->get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(kalman_filter_param->get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(kalman_filter_param->get_P0().isApprox(expected_initial_state_covariance));
}

TEST(KalmanFilterParamLoaderTest, WithoutInitialState) {
  // Set expected parameter values
  int state_size = 3;
  int input_size = 2;
  int output_size = 1;

  std::vector<double> flat_A, flat_B, flat_C, flat_D, flat_initial_state;
  std::vector<double> flat_measurement_noise_covariance,
    flat_process_noise_covariance,
    flat_initial_state_covariance;

  Eigen::MatrixXd expected_A(state_size, state_size);
  expected_A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  Eigen::MatrixXd expected_B(state_size, input_size);
  expected_B << 10, 11, 12, 13, 14, 14;

  Eigen::MatrixXd expected_C(output_size, state_size);
  expected_C << 14, 15, 16;

  Eigen::MatrixXd expected_D(output_size, input_size);
  expected_D << 17, 18;

  Eigen::VectorXd expected_initial_state(state_size);
  expected_initial_state << 19, 20, 21;

  for (int i = 0; i < expected_A.size(); ++i) {
    flat_A.push_back(expected_A.data()[i]);
  }

  for (int i = 0; i < expected_B.size(); ++i) {
    flat_B.push_back(expected_B.data()[i]);
  }

  for (int i = 0; i < expected_C.size(); ++i) {
    flat_C.push_back(expected_C.data()[i]);
  }

  for (int i = 0; i < expected_D.size(); ++i) {
    flat_D.push_back(expected_D.data()[i]);
  }

  for (int i = 0; i < expected_initial_state.size(); ++i) {
    flat_initial_state.push_back(expected_initial_state.data()[i]);
  }

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters();


  node_options.append_parameter_override("state_size", 3);
  node_options.append_parameter_override("input_size", 2);
  node_options.append_parameter_override("output_size", 1);
  node_options.append_parameter_override("state_transition_matrix", flat_A);
  node_options.append_parameter_override("input_matrix", flat_B);
  node_options.append_parameter_override("output_matrix", flat_C);
  node_options.append_parameter_override("feedforward_matrix", flat_D);
  // node_options.append_parameter_override("initial_state", flat_initial_state);

  Eigen::MatrixXd expected_process_noise_covariance(state_size, state_size);
  expected_process_noise_covariance << 10, 0.1, 0.1, 0.1, 10, 0.1, 0.1, 0.1, 10;

  for (int i = 0; i < expected_process_noise_covariance.size(); ++i) {
    flat_process_noise_covariance.push_back(expected_process_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_measurement_noise_covariance(output_size, output_size);
  expected_measurement_noise_covariance << 3;

  for (int i = 0; i < expected_measurement_noise_covariance.size(); ++i) {
    flat_measurement_noise_covariance.push_back(expected_measurement_noise_covariance.data()[i]);
  }

  Eigen::MatrixXd expected_initial_state_covariance(state_size, state_size);
  expected_initial_state_covariance << 2, 0.1, 0.1, 0.1, 2, 0.1, 0.1, 0.1, 2;

  for (int i = 0; i < expected_initial_state_covariance.size(); ++i) {
    flat_initial_state_covariance.push_back(expected_initial_state_covariance.data()[i]);
  }


  node_options.append_parameter_override("process_noise_covariance", flat_process_noise_covariance);
  node_options.append_parameter_override(
    "measurement_noise_covariance",
    flat_measurement_noise_covariance);
  node_options.append_parameter_override("initial_state_covariance", flat_initial_state_covariance);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "kalman_filter_param_node", node_options);

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserverParam>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserverParam>(
      "state_observers",
      "state_observer::StateObserverParam"));
  std::shared_ptr<state_observer::StateObserverParam> state_observer_param;

  try {
    state_observer_param =
      state_observer_loader->createSharedInstance(
      "state_observer::KalmanFilterParam");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  state_observer_param->initialize(node);
  ASSERT_TRUE(state_observer_param->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer_param->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer_param->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer_param->get_D().isApprox(expected_D));
  ASSERT_EQ(state_observer_param->get_initial_state().size(), 0);

  std::shared_ptr<state_observer::KalmanFilterParam> kalman_filter_param =
    std::dynamic_pointer_cast<state_observer::KalmanFilterParam>(state_observer_param);

  ASSERT_TRUE(kalman_filter_param->get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(kalman_filter_param->get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(kalman_filter_param->get_P0().isApprox(expected_initial_state_covariance));
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
