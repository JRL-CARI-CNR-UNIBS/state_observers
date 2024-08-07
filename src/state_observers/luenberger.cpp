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

#include "state_observers/luenberger.hpp"
#include <iostream>

namespace state_observer
{

Luenberger::Luenberger(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & L)
: StateObserver(A, B, C, D, initial_state)
{
  if (L.rows() != A.rows() || L.cols() != C.rows()) {
    throw std::invalid_argument("Observer gain matrix must have dimensions n x q.");
  }
  L_ = L;
}

Luenberger::Luenberger(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & L)
: StateObserver(A, B, C, initial_state), L_(L)
{
  if (L.rows() != A.rows() || L.cols() != C.rows()) {
    throw std::invalid_argument("Observer gain matrix must have dimensions n x q.");
  }
  L_ = L;
}

Luenberger::Luenberger(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & L)
: StateObserver(A, B, C), L_(L)
{
  if (L.rows() != A.rows() || L.cols() != C.rows()) {
    throw std::invalid_argument("Observer gain matrix must have dimensions n x q.");
  }
  L_ = L;
}

void
Luenberger::set_parameters(const StateObserverParam::SharedPtr state_observer_params)
{
  if (state_observer_params->get_type() != "Luenberger") {
    throw std::invalid_argument("State observer type must be Luenberger.");
  }
  LuenbergerParam::SharedPtr luenberger_params = std::dynamic_pointer_cast<LuenbergerParam>(
    state_observer_params);

  try {
    StateObserver::dimensions_check(
      luenberger_params->get_A(), luenberger_params->get_B(),
      luenberger_params->get_C(), luenberger_params->get_D(),
      luenberger_params->get_initial_state());
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  A_ = luenberger_params->get_A();
  B_ = luenberger_params->get_B();
  C_ = luenberger_params->get_C();
  D_ = luenberger_params->get_D();
  StateObserver::initialize(luenberger_params->get_initial_state());
  y_ = Eigen::VectorXd::Zero(C_.rows());

  try {
    set_observer_gain(luenberger_params->get_observer_gain());
  } catch (const std::invalid_argument & e) {
    throw e;
  }
}

void Luenberger::set_observer_gain(const Eigen::MatrixXd & L)
{
  if (L.rows() != A_.rows() || L.cols() != C_.rows()) {
    throw std::invalid_argument("Observer gain matrix must have dimensions n x q.");
  }
  L_ = L;
}

Eigen::MatrixXd Luenberger::update(const Eigen::VectorXd & measurement)
{
  return update(measurement, Eigen::VectorXd::Zero(B_.cols()));
}

Eigen::MatrixXd Luenberger::update(
  const Eigen::VectorXd & measurement,
  const Eigen::VectorXd & input)
{
  if (measurement.size() != C_.rows()) {
    throw std::invalid_argument("Measurement vector must have size q.");
  }
  if (input.size() != B_.cols()) {
    throw std::invalid_argument("Input vector must have size p.");
  }

  y_ = C_ * x_ + D_ * input;
  x_ = A_ * x_ + B_ * input + L_ * (measurement - y_);
  std::cerr << "x_ = " << x_[0] << std::endl;
  std::cerr << "L_ = " << L_[0] << std::endl;
  std::cerr << "measurement = " << measurement[0] << std::endl;
  std::cerr << "y_ = " << y_[0] << std::endl;

  return y_;
}

}  // namespace state_observer
