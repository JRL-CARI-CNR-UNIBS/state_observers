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

namespace state_observer
{

class StateObserver
{
public:
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
    const Eigen::VectorXd & initial_state);
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C,
    const Eigen::VectorXd & initial_state);
  StateObserver(
    const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C);

  virtual ~StateObserver() {}
  void initialize(const Eigen::VectorXd & initial_state);
  void set_initial_state(const Eigen::VectorXd & initial_state);
  virtual Eigen::VectorXd open_loop_update();
  virtual Eigen::MatrixXd update(const Eigen::VectorXd & measurement) = 0;
  virtual Eigen::MatrixXd update(
    const Eigen::VectorXd & measurement,
    const Eigen::VectorXd & input) = 0;
  inline Eigen::VectorXd get_state() const {return x_;}
  inline Eigen::VectorXd get_output() const {return C_ * x_;}
  inline bool is_initialized() const {return initialized_;}

protected:
  Eigen::MatrixXd A_, B_, C_, D_;
  Eigen::VectorXd x_, y_;
  bool initialized_ = false;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS__STATE_OBSERVER_HPP_
