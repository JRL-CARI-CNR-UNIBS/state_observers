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

#include "state_observers/kalman_filter.hpp"
#include "state_observers/luenberger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

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


TEST(KalmanFilterTest, Ros2PublisherIntegration) {
  // Define system model matrices
  double dt = 0.1; // Sample time
  Eigen::MatrixXd A(1, 1);
  A << 1;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(1, 1); // Select position as measurement
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

  // Define process and measurement noise covariance matrices (Q and R)
  // Choose appropriate values for process and measurement noise variances
  double process_noise_variance = 0.01;
  double measurement_noise_variance = 1;
  Eigen::MatrixXd Q = process_noise_variance * Eigen::MatrixXd::Identity(1, 1);
  Eigen::MatrixXd R = measurement_noise_variance * Eigen::MatrixXd::Identity(1, 1);

  // Initialize Kalman filter
  Eigen::VectorXd initial_state(1);
  initial_state << 0; // Initial position and velocity
  state_observer::KalmanFilter kf(A, B, C, D, initial_state, Q, R);

  // Initialize ROS 2 node
  rclcpp::init(0, nullptr);

  // Create ROS 2 publisher
  auto node = std::make_shared<rclcpp::Node>("kalman_filter_test_publisher");
  auto publisher_filtered = node->create_publisher<std_msgs::msg::Float32>("/filtered", 10);
  auto publisher_nominal = node->create_publisher<std_msgs::msg::Float32>("/nominal", 10);
  auto publisher_measure = node->create_publisher<std_msgs::msg::Float32>("/measure", 10);
  auto publisher_measure_luemberger = node->create_publisher<std_msgs::msg::Float32>(
    "/luemberger",
    10);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.2, 0.2);
  rclcpp::Rate loop_rate(5);
  // Simulate system dynamics and update filter with measurements
  int z = 0;
  while (true) {
    state_observer::KalmanFilter kf(A, B, C, D, initial_state, Q, R);
    Eigen::MatrixXd L(1, 1);
    L << 0.1;
    state_observer::Luenberger luenberger(A, B, C, D, initial_state, L);
    kf.initialize(initial_state);
    luenberger.initialize(initial_state);
    for (int i = 0; i < 100; ++i) {

      // Generate simulated measurement with noise
      double measurement_value = z + dis(gen);


      // Update Kalman filter with measurement
      Eigen::VectorXd measurement_vector(1);
      measurement_vector << measurement_value;
      kf.update(measurement_vector);
      luenberger.update(measurement_vector);

      // Simulate system dynamics (e.g., constant velocity motion)
      // true_position = kf.get_state()[0] + kf.get_state()[1] * dt;
      // true_velocity = kf.get_state()[1];

      // Publish measurement to ROS 2 topic
      auto msg = std::make_shared<std_msgs::msg::Float32>();
      msg->data = kf.get_state()[0];
      publisher_filtered->publish(*msg);
      auto msg_nominal = std::make_shared<std_msgs::msg::Float32>();
      msg_nominal->data = z;
      publisher_nominal->publish(*msg_nominal);
      auto msg_measure = std::make_shared<std_msgs::msg::Float32>();
      msg_measure->data = measurement_value;
      publisher_measure->publish(*msg_measure);
      auto msg_measure_luenberger = std::make_shared<std_msgs::msg::Float32>();
      msg_measure_luenberger->data = luenberger.get_state()[0];
      publisher_measure_luemberger->publish(*msg_measure_luenberger);

      loop_rate.sleep();
    }
    z++;
  }
  // Check the final estimated state
  // Add appropriate assertions based on expected behavior

  // Shutdown ROS 2 node
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
