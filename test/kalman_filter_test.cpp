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

#include "state_observers/kalman_filter.hpp"
#include "state_observers/luenberger.hpp"
#include "state_observers_param/kalman_filter_param.hpp"

TEST(KalmanFilterTest, ConstructorWithInitialState) {
  // Define matrices
  Eigen::MatrixXd A(2, 2);
  A << 1, 1, 0, 1;
  Eigen::MatrixXd B(2, 1);
  B << 0, 1;
  Eigen::MatrixXd C(1, 2);
  C << 1, 0;
  Eigen::MatrixXd Q(2, 2);
  Q << 0.1, 0, 0, 0.1;
  Eigen::MatrixXd R(1, 1);
  R << 0.1;
  Eigen::VectorXd initial_state(2);
  initial_state << 0, 0;

  // Create KalmanFilter object
  state_observer::KalmanFilter kf(A, B, C, initial_state, Q, R);

  // Check initial state
  Eigen::VectorXd state = kf.get_state();
  ASSERT_EQ(state.size(), 2);
  EXPECT_DOUBLE_EQ(state[0], initial_state[0]);
  EXPECT_DOUBLE_EQ(state[1], initial_state[1]);

  // Check initial covariance matrix
  Eigen::VectorXd variance = kf.get_state_variance();
  ASSERT_EQ(variance.size(), 2);
  EXPECT_DOUBLE_EQ(variance[0], 0);
  EXPECT_DOUBLE_EQ(variance[1], 0);
}

double gaussian_noise_generator(double variance)
{
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0.0, std::sqrt(variance));
  return distribution(generator);
}

TEST(KalmanFilterTest, InternalMatrices)
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

  Eigen::MatrixXd expected_process_noise_covariance(state_size, state_size);
  expected_process_noise_covariance << 10, 0.1, 0.1, 0.1, 10, 0.1, 0.1, 0.1, 10;

  Eigen::MatrixXd expected_measurement_noise_covariance(output_size, output_size);
  expected_measurement_noise_covariance << 3;

  Eigen::MatrixXd expected_initial_state_covariance(state_size, state_size);
  expected_initial_state_covariance << 2, 0.1, 0.1, 0.1, 2, 0.1, 0.1, 0.1, 2;

  state_observer::KalmanFilter kf(expected_A,
    expected_B,
    expected_C,
    expected_D, expected_initial_state, expected_process_noise_covariance,
    expected_measurement_noise_covariance, expected_initial_state_covariance);

  ASSERT_TRUE(kf.get_A().isApprox(expected_A));
  ASSERT_TRUE(kf.get_B().isApprox(expected_B));
  ASSERT_TRUE(kf.get_C().isApprox(expected_C));
  ASSERT_TRUE(kf.get_D().isApprox(expected_D));
  ASSERT_TRUE(kf.get_initial_state().isApprox(expected_initial_state));
  ASSERT_TRUE(kf.get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(kf.get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(kf.get_P0().isApprox(expected_initial_state_covariance));
}

TEST(KalmanFilterTest, KalmanFilterPlugin)
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

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserver>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserver>(
      "state_observers",
      "state_observer::StateObserver"));
  std::shared_ptr<state_observer::StateObserver> state_observer;

  try {
    state_observer =
      state_observer_loader->createSharedInstance(
      "state_observer::KalmanFilter");
    std::cerr << "state_observer::KalmanFilter" << std::endl;
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
  node_options.append_parameter_override("process_noise_covariance", flat_process_noise_covariance);
  node_options.append_parameter_override(
    "measurement_noise_covariance",
    flat_measurement_noise_covariance);
  node_options.append_parameter_override("initial_state_covariance", flat_initial_state_covariance);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "kalman_filter_param_node", node_options);

  std::shared_ptr<state_observer::KalmanFilterParam> kalman_filter_param =
    std::make_shared<state_observer::KalmanFilterParam>();
  kalman_filter_param->initialize(node);

  state_observer->set_parameters(
    kalman_filter_param);

  ASSERT_TRUE(state_observer->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer->get_D().isApprox(expected_D));
  ASSERT_TRUE(state_observer->get_initial_state().isApprox(expected_initial_state));

  std::shared_ptr<state_observer::KalmanFilter> kf =
    std::dynamic_pointer_cast<state_observer::KalmanFilter>(state_observer);

  ASSERT_TRUE(kf->get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(kf->get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(kf->get_P0().isApprox(expected_initial_state_covariance));
}

TEST(KalmanFilterTest, KalmanFilterAndParamPlugin)
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

  std::shared_ptr<pluginlib::ClassLoader<state_observer::StateObserver>> state_observer_loader;
  state_observer_loader.reset(
    new pluginlib::ClassLoader<state_observer::StateObserver>(
      "state_observers",
      "state_observer::StateObserver"));
  std::shared_ptr<state_observer::StateObserver> state_observer;

  try {
    state_observer =
      state_observer_loader->createSharedInstance(
      "state_observer::KalmanFilter");
    std::cerr << "state_observer::KalmanFIilter" << std::endl;
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
      "state_observer::KalmanFilterParam");
    std::cerr << "state_observer::KalmanFIilterParam" << std::endl;
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
  node_options.append_parameter_override("process_noise_covariance", flat_process_noise_covariance);
  node_options.append_parameter_override(
    "measurement_noise_covariance",
    flat_measurement_noise_covariance);
  node_options.append_parameter_override("initial_state_covariance", flat_initial_state_covariance);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "kalman_filter_param_node", node_options);

  state_observer_param->initialize(node);

  state_observer->set_parameters(
    state_observer_param);

  ASSERT_TRUE(state_observer->get_A().isApprox(expected_A));
  ASSERT_TRUE(state_observer->get_B().isApprox(expected_B));
  ASSERT_TRUE(state_observer->get_C().isApprox(expected_C));
  ASSERT_TRUE(state_observer->get_D().isApprox(expected_D));
  ASSERT_TRUE(state_observer->get_initial_state().isApprox(expected_initial_state));

  std::shared_ptr<state_observer::KalmanFilter> kf =
    std::dynamic_pointer_cast<state_observer::KalmanFilter>(state_observer);

  ASSERT_TRUE(kf->get_Q().isApprox(expected_process_noise_covariance));
  ASSERT_TRUE(kf->get_R().isApprox(expected_measurement_noise_covariance));
  ASSERT_TRUE(kf->get_P0().isApprox(expected_initial_state_covariance));
}

// TEST(KalmanFilterTest, Ros2PublisherIntegration) {
//   // Define system model matrices
//   double dt = 0.1; // Sample time
//   Eigen::MatrixXd A(1, 1);
//   A << 1;
//   Eigen::MatrixXd B = Eigen::MatrixXd::Zero(1, 1);
//   Eigen::MatrixXd C = Eigen::MatrixXd::Identity(1, 1); // Select position as measurement
//   Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

//   // Define process and measurement noise covariance matrices (Q and R)
//   // Choose appropriate values for process and measurement noise variances
//   double process_noise_variance = 0.01;
//   double measurement_noise_variance = 1;
//   Eigen::MatrixXd Q = process_noise_variance * Eigen::MatrixXd::Identity(1, 1);
//   Eigen::MatrixXd R = measurement_noise_variance * Eigen::MatrixXd::Identity(1, 1);

//   // Initialize Kalman filter
//   Eigen::VectorXd initial_state(1);
//   initial_state << 0; // Initial position and velocity
//   state_observer::KalmanFilter kf(A, B, C, D, initial_state, Q, R);

//   // Initialize ROS 2 node
//   rclcpp::init(0, nullptr);

//   // Create ROS 2 publisher
//   auto node = std::make_shared<rclcpp::Node>("kalman_filter_test_publisher");
//   auto publisher_filtered = node->create_publisher<std_msgs::msg::Float32>("/filtered", 10);
//   auto publisher_nominal = node->create_publisher<std_msgs::msg::Float32>("/nominal", 10);
//   auto publisher_measure = node->create_publisher<std_msgs::msg::Float32>("/measure", 10);
//   auto publisher_measure_luemberger = node->create_publisher<std_msgs::msg::Float32>(
//     "/luemberger",
//     10);

//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_real_distribution<> dis(-0.2, 0.2);
//   rclcpp::Rate loop_rate(5);
//   // Simulate system dynamics and update filter with measurements
//   int z = 0;
//   while (true) {
//     state_observer::KalmanFilter kf(A, B, C, D, initial_state, Q, R);
//     Eigen::MatrixXd L(1, 1);
//     L << 0.1;
//     state_observer::Luenberger luenberger(A, B, C, D, initial_state, L);
//     kf.initialize(initial_state);
//     luenberger.initialize(initial_state);
//     for (int i = 0; i < 100; ++i) {

//       // Generate simulated measurement with noise
//       double measurement_value = z + dis(gen);


//       // Update Kalman filter with measurement
//       Eigen::VectorXd measurement_vector(1);
//       measurement_vector << measurement_value;
//       kf.update(measurement_vector);
//       luenberger.update(measurement_vector);

//       // Simulate system dynamics (e.g., constant velocity motion)
//       // true_position = kf.get_state()[0] + kf.get_state()[1] * dt;
//       // true_velocity = kf.get_state()[1];

//       // Publish measurement to ROS 2 topic
//       auto msg = std::make_shared<std_msgs::msg::Float32>();
//       msg->data = kf.get_state()[0];
//       publisher_filtered->publish(*msg);
//       auto msg_nominal = std::make_shared<std_msgs::msg::Float32>();
//       msg_nominal->data = z;
//       publisher_nominal->publish(*msg_nominal);
//       auto msg_measure = std::make_shared<std_msgs::msg::Float32>();
//       msg_measure->data = measurement_value;
//       publisher_measure->publish(*msg_measure);
//       auto msg_measure_luenberger = std::make_shared<std_msgs::msg::Float32>();
//       msg_measure_luenberger->data = luenberger.get_state()[0];
//       publisher_measure_luemberger->publish(*msg_measure_luenberger);

//       loop_rate.sleep();
//     }
//     z++;
//   }
//   // Check the final estimated state
//   // Add appropriate assertions based on expected behavior

//   // Shutdown ROS 2 node
//   rclcpp::shutdown();
// }

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
