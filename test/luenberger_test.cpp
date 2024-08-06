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
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <pluginlib/class_loader.hpp>

#include "state_observers/luenberger.hpp"
#include "state_observers_param/luenberger_param.hpp"

TEST(LuenbergerTest, ConstructorWithInitialState) {
  // Define matrices
  Eigen::MatrixXd A(2, 2);
  A << 1, 1, 0, 1;
  Eigen::MatrixXd B(2, 1);
  B << 0, 1;
  Eigen::MatrixXd C(1, 2);
  C << 1, 0;
  Eigen::MatrixXd L(2, 1);
  L << 1, 2;
  Eigen::VectorXd initial_state(2);
  initial_state << 0, 0;

  // Create Luenberger object
  state_observer::Luenberger lu(A, B, C, initial_state, L);

  // Check initial state
  Eigen::VectorXd state = lu.get_state();
  ASSERT_EQ(state.size(), 2);
  EXPECT_DOUBLE_EQ(state[0], initial_state[0]);
  EXPECT_DOUBLE_EQ(state[1], initial_state[1]);

  // Check L gain matrix
  Eigen::VectorXd L_gain = lu.get_observer_gain();
  ASSERT_EQ(L_gain.size(), 2);
  EXPECT_DOUBLE_EQ(L_gain[0], 1);
  EXPECT_DOUBLE_EQ(L_gain[1], 2);
}


TEST(LuenbergerTest, InternalMatrices)
{
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

  Eigen::MatrixXd L_gain(state_size, output_size);
  L_gain << 5, 2, 9;

  state_observer::Luenberger lu(expected_A,
    expected_B,
    expected_C,
    expected_D, expected_initial_state, L_gain);

  ASSERT_TRUE(lu.get_A().isApprox(expected_A));
  ASSERT_TRUE(lu.get_B().isApprox(expected_B));
  ASSERT_TRUE(lu.get_C().isApprox(expected_C));
  ASSERT_TRUE(lu.get_D().isApprox(expected_D));
  ASSERT_TRUE(lu.get_initial_state().isApprox(expected_initial_state));
  ASSERT_TRUE(lu.get_observer_gain().isApprox(L_gain));
}

TEST(LuenbergerTest, LuenbergerTestPlugin)
{
  // Set expected parameter values
  int state_size = 3;
  int input_size = 2;
  int output_size = 1;

  std::vector<double> flat_A, flat_B, flat_C, flat_D, flat_initial_state;
  std::vector<double> flat_L;

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

  Eigen::MatrixXd expected_L_gain(state_size, output_size);
  expected_L_gain << 5, 2, 9;

  for (int i = 0; i < expected_L_gain.size(); ++i) {
    flat_L.push_back(expected_L_gain.data()[i]);
  }

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserver>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserver>(
      "state_observers",
      "state_observer::StateObserver"));
  std::shared_ptr<state_observer::StateObserver> state_observer;

  try {
    state_observer =
      state_observer_loader->createSharedInstance(
      "state_observer::Luenberger");
    std::cerr << "state_observer::Luenberger" << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
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
  node_options.append_parameter_override("Luenberger_gain", flat_L);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "luenberger_param_node_test", node_options);

  std::shared_ptr<state_observer::LuenbergerParam> luenberger_param =
    std::make_shared<state_observer::LuenbergerParam>();
  luenberger_param->initialize(node);

  state_observer->set_parameters(
    luenberger_param);

  ASSERT_TRUE(state_observer->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer->get_D().isApprox(expected_D));
  ASSERT_TRUE(state_observer->get_initial_state().isApprox(expected_initial_state));

  std::shared_ptr<state_observer::Luenberger> kf =
    std::dynamic_pointer_cast<state_observer::Luenberger>(state_observer);

  ASSERT_TRUE(kf->get_observer_gain().isApprox(expected_L_gain));
}

TEST(KalmanFilterTest, LuenbergerAndParamPlugin)
{
  // Set expected parameter values
  int state_size = 3;
  int input_size = 2;
  int output_size = 1;

  std::vector<double> flat_A, flat_B, flat_C, flat_D, flat_initial_state;
  std::vector<double> flat_L;

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

  Eigen::MatrixXd expected_L_gain(state_size, output_size);
  expected_L_gain << 5, 2, 9;

  for (int i = 0; i < expected_L_gain.size(); ++i) {
    flat_L.push_back(expected_L_gain.data()[i]);
  }

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserver>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserver>(
      "state_observers",
      "state_observer::StateObserver"));
  std::shared_ptr<state_observer::StateObserver> state_observer;

  try {
    state_observer =
      state_observer_loader->createSharedInstance(
      "state_observer::Luenberger");
    std::cerr << "state_observer::Luenberger" << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserverParam>>
  state_observer_param_loader;
  state_observer_param_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserverParam>(
      "state_observers",
      "state_observer::StateObserverParam"));
  std::shared_ptr<state_observer::StateObserverParam> state_observer_param;

  try {
    state_observer_param =
      state_observer_param_loader->createSharedInstance(
      "state_observer::LuenbergerParam");
    std::cerr << "state_observer::LuenbergerParam" << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
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
  node_options.append_parameter_override("Luenberger_gain", flat_L);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "luenberger_test_node", node_options);

  state_observer_param->initialize(node);

  state_observer->set_parameters(
    state_observer_param);

  ASSERT_TRUE(state_observer->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer->get_D().isApprox(expected_D));
  ASSERT_TRUE(state_observer->get_initial_state().isApprox(expected_initial_state));

  std::shared_ptr<state_observer::Luenberger> lu =
    std::dynamic_pointer_cast<state_observer::Luenberger>(state_observer);

  ASSERT_TRUE(lu->get_observer_gain().isApprox(expected_L_gain));
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}