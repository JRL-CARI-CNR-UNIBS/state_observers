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

#ifndef STATE_OBSERVERS__STATE_OBSERVER_HPP_
#define STATE_OBSERVERS__STATE_OBSERVER_HPP_

#include <Eigen/Dense>
#include <state_observers_param/state_observer_param.hpp>

namespace state_observer
{

/**
 * @class StateObserver
 * @brief A generic state observer class for linear systems.
 *
 * The `StateObserver` class provides methods for initializing and updating the state of a linear system observer
 * defined by state-space matrices (A, B, C, D) and an initial state vector.
 */
class StateObserver
{
public:
  /**
   * @brief Constructor with full state-space matrices and initial state.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param D Feedthrough matrix (q x p).
   * @param initial_state Initial state vector (n x 1).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state);

  /**
   * @brief Constructor without feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   * @param initial_state Initial state vector (n x 1).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state);

  /**
   * @brief Constructor without initial state and feedthrough matrix D.
   *
   * @param A State transition matrix (n x n).
   * @param B Input matrix (n x p).
   * @param C Output matrix (q x n).
   *
   * @throws std::invalid_argument if dimensions are inconsistent.
   */
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C);

  /**
   * @brief Default constructor.
   */
  StateObserver() {}

  ~StateObserver() {}

  /**
   * @brief Set the parameters of the state observer.
   *
   * @param state_observer_params Shared pointer to `StateObserverParam`.
   */
  virtual void set_parameters(const StateObserverParam::SharedPtr state_observer_params) {}
  // TODO(@samu) Pure virtual and mandatory in inheritance

  /**
   * @brief Initialize the state observer with an initial state.
   *
   * @param initial_state Initial state vector (n x 1).
   *
   * @throws std::invalid_argument if `initial_state` size is incorrect.
   */
  void initialize(const Eigen::VectorXd & initial_state);

  /**
 * @brief Set the state-space model (A, B, C, D).
 *
 * This method allows late initialization of the system matrices, useful when using the default constructor.
 * It performs a dimensional consistency check.
 *
 * @param A State transition matrix (n x n).
 * @param B Input matrix (n x p).
 * @param C Output matrix (q x n).
 * @param D Feedthrough matrix (q x p).
 * @param initial_state Optional initial state vector (n x 1).
 *
 * @throws std::invalid_argument if dimensions are inconsistent.
 */
  void set_model(
    const Eigen::MatrixXd & A,
    const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::MatrixXd & D);

  /**
   * @brief Perform an open-loop update of the observer.
   *
   * Updates the state `x_` using the state transition matrix `A_` without considering any input or measurement.
   *
   * @return Updated output vector `y_`.
   */
  virtual Eigen::VectorXd open_loop_update();

  /**
   * @brief Update the observer with a measurement.
   *
   * @param measurement Measurement vector.
   * @return Updated state estimate.
   */
  virtual Eigen::MatrixXd update(const Eigen::VectorXd & measurement) = 0;

  /**
   * @brief Update the observer with a measurement and input.
   *
   * @param measurement Measurement vector.
   * @param input Input vector.
   * @return Updated state estimate.
   */
  virtual Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) = 0;

  /**
   * @brief Get the current state estimate.
   *
   * @return Current state vector `x_`.
   */
  inline Eigen::VectorXd get_state() const {return x_;}

  /**
   * @brief Get the current output estimate.
   *
   * @return Current output vector `y_`.
   */
  inline Eigen::VectorXd get_output() const {return C_ * x_;}

  inline bool is_initialized() const {return initialized_;}

  /**
   * @brief Get the variance of the state estimate.
   *
   * @throws std::runtime_error if not implemented.
   * @return State variance vector.
   */
  virtual Eigen::VectorXd get_state_variance() {throw std::runtime_error("Not implemented.");}
  void set_state_transition_matrix(const Eigen::MatrixXd & A);

  inline Eigen::MatrixXd get_A() const {return A_;}
  inline Eigen::MatrixXd get_B() const {return B_;}
  inline Eigen::MatrixXd get_C() const {return C_;}
  inline Eigen::MatrixXd get_D() const {return D_;}
  inline Eigen::VectorXd get_initial_state() const {return x_;}

protected:
  Eigen::MatrixXd A_, B_, C_, D_;
  Eigen::VectorXd x_, y_;
  bool initialized_ = false;

  void dimensions_check(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D
  );

  void dimensions_check(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state
  );
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS__STATE_OBSERVER_HPP_
