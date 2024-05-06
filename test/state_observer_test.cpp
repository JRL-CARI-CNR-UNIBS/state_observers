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

#include "state_observers/state_observer.hpp"
#include "state_observers/luenberger.hpp"
#include "state_observers/kalman_filter.hpp"

class StateObserverDummy : public state_observer::StateObserver
{
public:
  StateObserverDummy(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state)
  : state_observer::StateObserver(A, B, C, D, initial_state)
  {
  }

  Eigen::MatrixXd update(const Eigen::VectorXd & measurement) override
  {
    auto avoid_warning = measurement;
    return Eigen::MatrixXd();
  }
  Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) override
  {
    auto avoid_warning_measurement = measurement;
    auto avoid_warning_input = input;
    return Eigen::MatrixXd();
  }
};

// Test to verify that the constructor with correct dimensions A B C D matrixes
TEST(StateObserverTest, ConstructorTest) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1;
  D << 0;
  initial_state << 0, 0;

  ASSERT_NO_THROW(StateObserverDummy observer(A, B, C, D, initial_state));
}

// Test to verify the initialize method
TEST(StateObserverTest, InitializeTest) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1;
  D << 0;
  initial_state << 0, 0;

  StateObserverDummy observer(A, B, C, D, initial_state);
  Eigen::VectorXd new_initial_state(2);
  new_initial_state << 1, 1;

  observer.initialize(new_initial_state);

  ASSERT_EQ(observer.get_state(), new_initial_state);
}

// Test to verify the open_loop_update method
TEST(StateObserverTest, OpenLoopUpdateTest) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1;
  D << 0;
  initial_state << 0, 0;

  StateObserverDummy observer(A, B, C, D, initial_state);

  Eigen::VectorXd output = observer.open_loop_update();

  // Given the initial state, the output should be [0, 0]
  ASSERT_EQ(output(0), 0);
}

// Test to verify the correct functioning of the get_state and get_output methods
TEST(StateObserverTest, GetStateAndGetOutputTest) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1;
  D << 0;
  initial_state << 0, 0;

  StateObserverDummy observer(A, B, C, D, initial_state);

  Eigen::VectorXd state = observer.get_state();
  Eigen::VectorXd output = observer.get_output();

  // Given the initial state, both the state and the output should be [0, 0]
  ASSERT_EQ(state(0), 0);
  ASSERT_EQ(output(0), 0);
}

// Test constructor with wrong matrices dimensions  (A matrix)
TEST(StateObserverTest, ConstructorExceptionTestA) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1;
  D << 0;

  // Wrong A matrix dimension
  A.resize(3, 3);

  ASSERT_THROW(StateObserverDummy observer(A, B, C, D, initial_state), std::invalid_argument);
}

// Test constructor with wrong matrices dimensions (B matrix)
TEST(StateObserverTest, ConstructorExceptionTestB) {
  Eigen::MatrixXd A(2, 2), B(3, 1), C(1, 2), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0, 3;   // Wrong B matrix dimension
  C << 1, 1;
  D << 0;

  ASSERT_THROW(StateObserverDummy observer(A, B, C, D, initial_state), std::invalid_argument);
}

// Test constructor with wrong matrices dimensions (C matrix)
TEST(StateObserverTest, ConstructorExceptionTestC) {
  Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 4), D(1, 1);
  Eigen::VectorXd initial_state(2);
  A << 1, 0, 0, 1;
  B << 1, 0;
  C << 1, 1, 0, 0;  // Wrong C matrix dimension
  D << 0;
  ASSERT_THROW(StateObserverDummy observer(A, B, C, D, initial_state), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
