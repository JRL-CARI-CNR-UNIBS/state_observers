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

#include "state_observers/state_observer.hpp"

namespace state_observer
{

StateObserver::StateObserver(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
  const Eigen::VectorXd & initial_state)
{
  if (A.rows() != A.cols()) {
    throw std::invalid_argument("Matrix A must be square of size n x n.");
  }
  if (B.rows() != A.rows()) {
    throw std::invalid_argument("Matrix B must have dimensions n x p.");
  }
  if (C.cols() != A.rows()) {
    throw std::invalid_argument("Matrix C must have dimensions q x n.");
  }
  if (D.cols() != B.cols() || D.rows() != C.rows()) {
    throw std::invalid_argument("Matrix D must have dimensions q x p.");
  }
  if (initial_state.size() != A.rows()) {
    throw std::invalid_argument("Initial state vector must have size n.");
  }
  // if (y_.size() != C_.rows()) {
  //   throw std::invalid_argument("Initial output vector must have size q.");
  // }

  A_ = A;
  B_ = B;
  C_ = C;
  D_ = D;
  x_ = initial_state;
  y_ = Eigen::VectorXd::Zero(C_.rows());
}

StateObserver::StateObserver(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::VectorXd & initial_state)
: StateObserver(A, B, C, Eigen::MatrixXd::Zero(C.rows(), B.cols()), initial_state) {}

StateObserver::StateObserver(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C)
: StateObserver(A, B, C, Eigen::VectorXd::Zero(A.rows())) {}

void StateObserver::initialize(const Eigen::VectorXd & initial_state)
{
  if (initial_state.size() != x_.size()) {
    throw std::invalid_argument("Initial state vector must have size n.");
  }

  x_ = initial_state;
}

Eigen::VectorXd StateObserver::open_loop_update()
{
  x_ = A_ * x_;
  y_ = C_ * x_;
  return y_;
}

}  // namespace state_observer
