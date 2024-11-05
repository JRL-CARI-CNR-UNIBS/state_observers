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

#ifndef STATE_OBSERVERS_PARAM__STATE_OBSERVER_PARAM_HPP_
#define STATE_OBSERVERS_PARAM__STATE_OBSERVER_PARAM_HPP_

#include <Eigen/Dense>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace state_observer
{

/**
 * @class StateObserverParam
 * @brief Base class for state observer parameters.
 *
 * The `StateObserverParam` class provides methods to initialize and manage the parameters
 * required for a state observer, including state-space matrices and the initial state vector.
 */
class StateObserverParam
{
public:
  /**
   * @brief Default constructor.
   */
  StateObserverParam() {}

  /**
   * @brief Destructor.
   */
  virtual ~StateObserverParam() {}

  /**
   * @brief Initialize the parameters from a ROS 2 lifecycle node.
   *
   * @param node Shared pointer to an `rclcpp_lifecycle::LifecycleNode`.
   *
   * @throws std::runtime_error if parameter retrieval fails.
   */
  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Get the state transition matrix `A`.
   *
   * @return State transition matrix `A_` (n x n).
   */
  inline Eigen::MatrixXd get_state_transition_matrix() {return A_;}

  /**
   * @brief Get the input matrix `B`.
   *
   * @return Input matrix `B_` (n x p).
   */
  inline Eigen::MatrixXd get_input_matrix() {return B_;}

  /**
   * @brief Get the output matrix `C`.
   *
   * @return Output matrix `C_` (q x n).
   */
  inline Eigen::MatrixXd get_output_matrix() {return C_;}

  /**
   * @brief Get the feedforward matrix `D`.
   *
   * @return Feedforward matrix `D_` (q x p).
   */
  inline Eigen::MatrixXd get_feedforward_matrix() {return D_;}

  /**
   * @brief Get the initial state vector.
   *
   * @return Initial state vector `initial_state_` (n x 1).
   */
  inline Eigen::VectorXd get_initial_state() const {return initial_state_;}

  /**
   * @brief Get the state transition matrix `A`.
   *
   * @return State transition matrix `A_` (n x n).
   */
  inline Eigen::MatrixXd get_A() {return A_;}

  /**
   * @brief Get the input matrix `B`.
   *
   * @return Input matrix `B_` (n x p).
   */
  inline Eigen::MatrixXd get_B() {return B_;}

  /**
   * @brief Get the output matrix `C`.
   *
   * @return Output matrix `C_` (q x n).
   */
  inline Eigen::MatrixXd get_C() {return C_;}

  /**
   * @brief Get the feedforward matrix `D`.
   *
   * @return Feedforward matrix `D_` (q x p).
   */
  inline Eigen::MatrixXd get_D() {return D_;}

  /**
   * @brief Get the type of the state observer.
   *
   * @return A string representing the type of the state observer.
   *
   * @throws std::runtime_error if not implemented.
   */
  virtual std::string get_type() const {throw std::runtime_error("Not implemented.");}
  // TODO(@samu) pure virtual and dummy in test

  using SharedPtr = std::shared_ptr<StateObserverParam>;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  Eigen::MatrixXd A_, B_, C_, D_;
  Eigen::VectorXd initial_state_;

  void fully_declare_parameter(
    const std::string & param_name,
    const rclcpp::ParameterType type,
    const std::string & description);
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS_PARAM__STATE_OBSERVER_PARAM_HPP_
