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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace state_observer
{

class StateObserverParam
{
public:
  StateObserverParam() {}

  virtual ~StateObserverParam() {}
  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  inline Eigen::MatrixXd get_state_transition_matrix() {return A_;}
  inline Eigen::MatrixXd get_input_matrix() {return B_;}
  inline Eigen::MatrixXd get_output_matrix() {return C_;}
  inline Eigen::MatrixXd get_feedforward_matrix() {return D_;}
  inline Eigen::VectorXd get_initial_state() {return initial_state_;}
  inline Eigen::MatrixXd get_A() {return A_;}
  inline Eigen::MatrixXd get_B() {return B_;}
  inline Eigen::MatrixXd get_C() {return C_;}
  inline Eigen::MatrixXd get_D() {return D_;}

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
