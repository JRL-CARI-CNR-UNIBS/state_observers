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


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
