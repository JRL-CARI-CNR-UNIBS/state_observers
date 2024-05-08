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

#ifndef STATE_OBSERVERS__KALMAN_FILTER_HPP_
#define STATE_OBSERVERS__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <state_observers/state_observer.hpp>

namespace state_observer
{

class KalmanFilter : public StateObserver
{
public:
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  Eigen::MatrixXd update(const Eigen::VectorXd & measurement) override;
  Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) override;
  Eigen::VectorXd open_loop_update() override;
  void update_process_covariance(const Eigen::MatrixXd & new_Q);
  void update_measurement_covariance(const Eigen::MatrixXd & new_R);
  void update_qr(const Eigen::MatrixXd & new_Q, const Eigen::MatrixXd & new_R);

protected:
  Eigen::VectorXd L_;
  Eigen::MatrixXd P_, K_, Q_, R_, I_;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS__KALMAN_FILTER_HPP_
