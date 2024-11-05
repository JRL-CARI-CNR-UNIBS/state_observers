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

#ifndef STATE_OBSERVERS__LUENBERGER_HPP_
#define STATE_OBSERVERS__LUENBERGER_HPP_

#include <Eigen/Dense>
#include <state_observers/state_observer.hpp>
#include <state_observers_param/state_observer_param.hpp>
#include <state_observers_param/luenberger_param.hpp>

namespace state_observer
{

/**
 * @class Luenberger
 * @brief Implementation of a Luenberger observer for linear systems.
 *
 * The `Luenberger` class extends the `StateObserver` base class to provide a Luenberger observer,
 * which estimates the state of a linear system using a specified observer gain matrix `L`.
 */
class Luenberger : public StateObserver
{
public:
  /**
   * @brief Constructor with full state-space matrices, initial state, and observer gain.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param D Feedthrough matrix (q x p).
   * @param initial_state Initial state vector (n x 1).
   * @param L Observer gain matrix (n x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  Luenberger(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & L);

  /**
   * @brief Constructor without feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param initial_state Initial state vector (n x 1).
   * @param L Observer gain matrix (n x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  Luenberger(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state,
    const Eigen::MatrixXd & L);

  /**
   * @brief Constructor without initial state and feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param L Observer gain matrix (n x q).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  Luenberger(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & L);

  /**
   * @brief Default constructor.
   */
  Luenberger() {}

  /**
   * @brief Destructor.
   */
  ~Luenberger() {}

  /**
   * @brief Set the parameters of the Luenberger observer.
   *
   * @param state_observer_params Shared pointer to `StateObserverParam`.
   */
  void set_parameters(const StateObserverParam::SharedPtr state_observer_params) override;

  /**
   * @brief Set the observer gain matrix `L`.
   *
   * @param L New observer gain matrix (n x q).
   *
   * @throws std::invalid_argument if dimensions are incorrect.
   */
  void set_observer_gain(const Eigen::MatrixXd & L);

  /**
   * @brief Update the observer with a measurement.
   *
   * @param measurement Measurement vector (q x 1).
   * @return Updated state estimate.
   */
  Eigen::MatrixXd update(const Eigen::VectorXd & measurement) override;

  /**
   * @brief Update the observer with a measurement and input.
   *
   * @param measurement Measurement vector (q x 1).
   * @param input Input vector (p x 1).
   * @return Updated state estimate.
   */
  Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) override;

  inline Eigen::MatrixXd get_observer_gain() const {return L_;}

protected:
  Eigen::VectorXd L_;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS__LUENBERGER_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  state_observer::Luenberger,
  state_observer::StateObserver)
