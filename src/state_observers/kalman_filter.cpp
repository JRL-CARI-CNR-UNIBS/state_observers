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

#include "state_observers/kalman_filter.hpp"

namespace state_observer
{

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
: StateObserver(A, B, C, D, initial_state)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
  P_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
}

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
: StateObserver(A, B, C, initial_state)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
  P_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
}

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
: StateObserver(A, B, C)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
  P_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
}

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P0)
: StateObserver(A, B, C, D, initial_state)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());

  if (P0.rows() != A.rows() || P0.cols() != A.cols()) {
    throw std::invalid_argument("Initial covariance matrix must have dimensions n x n.");
  }
  P_ = P0;
}

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::VectorXd & initial_state,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P0)
: StateObserver(A, B, C, initial_state)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());

  if (P0.rows() != A.rows() || P0.cols() != A.cols()) {
    throw std::invalid_argument("Initial covariance matrix must have dimensions n x n.");
  }
  P_ = P0;
}

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P0)
: StateObserver(A, B, C)
{
  try {
    update_qr(Q, R);
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());

  if (P0.rows() != A.rows() || P0.cols() != A.cols()) {
    throw std::invalid_argument("Initial covariance matrix must have dimensions n x n.");
  }
  P_ = P0;
}

void
KalmanFilter::set_parameters(const StateObserverParam::SharedPtr state_observer_params)
{
  if (state_observer_params->get_type() != "KalmanFilter") {
    throw std::invalid_argument("State observer type must be KalmanFilter.");
  }
  KalmanFilterParam::SharedPtr kalman_filter_params = std::dynamic_pointer_cast<KalmanFilterParam>(
    state_observer_params);

  try {
    StateObserver::dimensions_check(
      kalman_filter_params->get_A(), kalman_filter_params->get_B(),
      kalman_filter_params->get_C(), kalman_filter_params->get_D(),
      kalman_filter_params->get_initial_state());
  } catch (const std::invalid_argument & e) {
    throw e;
  }
  A_ = kalman_filter_params->get_A();
  B_ = kalman_filter_params->get_B();
  C_ = kalman_filter_params->get_C();
  D_ = kalman_filter_params->get_D();
  if (kalman_filter_params->get_initial_state().size() != 0) {
    StateObserver::initialize(kalman_filter_params->get_initial_state());
  }
  y_ = Eigen::VectorXd::Zero(C_.rows());

  try {
    update_qr(kalman_filter_params->get_Q(), kalman_filter_params->get_R());
  } catch (const std::invalid_argument & e) {
    throw e;
  }

  I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());

  auto P0 = kalman_filter_params->get_P0();

  if (P0.rows() != A_.rows() || P0.cols() != A_.cols()) {
    throw std::invalid_argument("Initial covariance matrix must have dimensions n x n.");
  }
  P_ = P0;
}

Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & measurement)
{
  return update(measurement, Eigen::VectorXd::Zero(B_.cols()));
}

Eigen::VectorXd KalmanFilter::open_loop_update()
{
  x_ = A_ * x_;
  y_ = C_ * x_;
  P_ = A_ * P_ * A_.transpose() + Q_;
  return y_;
}

Eigen::MatrixXd KalmanFilter::update(
  const Eigen::VectorXd & measurement,
  const Eigen::VectorXd & input)
{
  if (measurement.size() != C_.rows()) {
    throw std::invalid_argument("Measurement vector must have size q.");
  }
  if (input.size() != B_.cols()) {
    throw std::invalid_argument("Input vector must have size p.");
  }
  if (I_.size() == 0) {
    I_ = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
  }

  // Predict
  if (B_.size() != 0) {
    x_ = A_ * x_ + B_ * input;
  } else {
    x_ = A_ * x_;
  }
  P_ = A_ * P_ * A_.transpose() + Q_;

  // Update
  K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();


  x_ = x_ + K_ * (measurement - C_ * x_);
  P_ = (I_ - K_ * C_) * P_;

  if (D_.size() != 0) {
    y_ = C_ * x_ + D_ * input;
  } else {
    y_ = C_ * x_;
  }

  return y_;
}
void KalmanFilter::update_process_covariance(const Eigen::MatrixXd & new_Q)
{
  Q_ = new_Q;
}

void KalmanFilter::update_measurement_covariance(const Eigen::MatrixXd & new_R)
{
  R_ = new_R;
}

void KalmanFilter::update_qr(
  const Eigen::MatrixXd & new_Q,
  const Eigen::MatrixXd & new_R)
{
  if (new_Q.rows() != A_.rows() || new_Q.cols() != A_.cols()) {
    throw std::invalid_argument("Process covariance matrix must have dimensions n x n.");
  }
  if (new_R.rows() != C_.rows() || new_R.cols() != C_.rows()) {
    throw std::invalid_argument("Measurement covariance matrix must have dimensions q x q.");
  }
  Q_ = new_Q;
  R_ = new_R;
}
Eigen::VectorXd KalmanFilter::get_state_variance()
{
  return P_.diagonal();
}

void KalmanFilter::set_P0(const Eigen::MatrixXd P0)
{
  if (P0.rows() != A_.rows() || P0.cols() != A_.cols()) {
    throw std::invalid_argument("Initial covariance matrix must have dimensions n x n.");
  }
  P_ = P0;
}

}  // namespace state_observer
