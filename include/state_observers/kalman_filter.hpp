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
#include <state_observers_param/kalman_filter_param.hpp>
#include <state_observers_param/state_observer_param.hpp>

namespace state_observer
{

/**
 * @class KalmanFilter
 * @brief Implementation of a discrete-time Kalman Filter for linear systems.
 *
 * The `KalmanFilter` class extends the `StateObserver` base class to provide a Kalman Filter,
 * which estimates the state of a linear system by minimizing the covariance of the estimation error.
 */
class KalmanFilter : public StateObserver
{
public:
  /**
   * @brief Constructor with full state-space matrices, initial state, and covariance matrices.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param D Feedthrough matrix (q x p).
   * @param initial_state Initial state vector (n x 1).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  /**
   * @brief Constructor without feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param initial_state Initial state vector (n x 1).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  /**
   * @brief Constructor without initial state and feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  /**
   * @brief Constructor with initial error covariance matrix P0.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param D Feedthrough matrix (q x p).
   * @param initial_state Initial state vector (n x 1).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   * @param P0 Initial error covariance matrix (n x n).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & P0);

  /**
   * @brief Constructor without feedthrough matrix D and with initial error covariance matrix P0.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param initial_state Initial state vector (n x 1).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   * @param P0 Initial error covariance matrix (n x n).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & P0);

  /**
   * @brief Constructor without initial state, feedthrough matrix D, and with initial error covariance matrix P0.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param Q Process noise covariance matrix (n x n).
   * @param R Measurement noise covariance matrix (q x q).
   * @param P0 Initial error covariance matrix (n x n).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  KalmanFilter(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & P0);

  /**
   * @brief Default constructor.
   */
  KalmanFilter() {}

  /**
   * @brief Destructor.
   */
  ~KalmanFilter() {}

  /**
   * @brief Set the parameters of the Kalman Filter.
   *
   * @param state_observer_params Shared pointer to `StateObserverParam`.
   */
  void set_parameters(const StateObserverParam::SharedPtr state_observer_params) override;

  /**
   * @brief Update the Kalman Filter with a measurement.
   *
   * @param measurement Measurement vector (q x 1).
   * @return Updated state estimate.
   */
  Eigen::MatrixXd update(const Eigen::VectorXd & measurement) override;

  /**
   * @brief Update the Kalman Filter with a measurement and input.
   *
   * @param measurement Measurement vector (q x 1).
   * @param input Input vector (p x 1).
   * @return Updated state estimate.
   */
  Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) override;

  /**
   * @brief Perform an open-loop update of the Kalman Filter.
   *
   * @return Updated output vector `y_`.
   */
  Eigen::VectorXd open_loop_update() override;

  /**
   * @brief Update the process noise covariance matrix `Q`.
   *
   * @param new_Q New process noise covariance matrix (n x n).
   *
   * @throws std::invalid_argument if dimensions are incorrect.
   */
  void update_process_covariance(const Eigen::MatrixXd & new_Q);

  /**
   * @brief Update the measurement noise covariance matrix `R`.
   *
   * @param new_R New measurement noise covariance matrix (q x q).
   *
   * @throws std::invalid_argument if dimensions are incorrect.
   */
  void update_measurement_covariance(const Eigen::MatrixXd & new_R);

  /**
   * @brief Update both the process and measurement noise covariance matrices.
   *
   * @param new_Q New process noise covariance matrix (n x n).
   * @param new_R New measurement noise covariance matrix (q x q).
   *
   * @throws std::invalid_argument if dimensions are incorrect.
   */
  void update_qr(const Eigen::MatrixXd & new_Q, const Eigen::MatrixXd & new_R);

  /**
   * @brief Get the variance of the state estimate.
   *
   * @return State variance vector.
   */
  Eigen::VectorXd get_state_variance() override;

  /**
   * @brief Get the process noise covariance matrix `Q`.
   *
   * @return Matrix `Q_`.
   */
  inline Eigen::MatrixXd get_Q() const {return Q_;}

  /**
   * @brief Get the measurement noise covariance matrix `R`.
   *
   * @return Matrix `R_`.
   */
  inline Eigen::MatrixXd get_R() const {return R_;}

  /**
   * @brief Get the initial error covariance matrix `P0`.
   *
   * @return Matrix `P_`.
   */
  inline Eigen::MatrixXd get_P0() const {return P_;}

  /**
   * @brief Set the initial error covariance matrix `P0`.
   *
   */
  void set_P0(const Eigen::MatrixXd P0);

protected:
  Eigen::VectorXd L_;
  Eigen::MatrixXd P_, K_, Q_, R_, I_;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS__KALMAN_FILTER_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  state_observer::KalmanFilter,
  state_observer::StateObserver)
