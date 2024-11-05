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

#ifndef STATE_OBSERVERS_PARAM__KALMAN_FILTER_PARAM_HPP_
#define STATE_OBSERVERS_PARAM__KALMAN_FILTER_PARAM_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "state_observers_param/state_observer_param.hpp"


namespace state_observer
{

/**
 * @class KalmanFilterParam
 * @brief Parameters for the Kalman Filter observer.
 *
 * The `KalmanFilterParam` class extends `StateObserverParam` to include
 * the process noise covariance matrix `Q`, measurement noise covariance matrix `R`,
 * and the initial error covariance matrix `P0` specific to the Kalman Filter observer.
 */
class KalmanFilterParam : public StateObserverParam
{
public:
  /**
   * @brief Default constructor.
   */
  KalmanFilterParam() {}

  /**
   * @brief Destructor.
   */
  virtual ~KalmanFilterParam() {}

  /**
   * @brief Initialize the parameters from a ROS 2 lifecycle node.
   *
   * This method reads the parameters from the provided node and initializes
   * the state-space matrices and covariance matrices.
   *
   * @param node Shared pointer to an `rclcpp_lifecycle::LifecycleNode`.
   *
   * @throws std::runtime_error if parameter retrieval fails.
   */
  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Get the measurement noise covariance matrix `R`.
   *
   * @return Measurement noise covariance matrix `R_` (q x q).
   */
  Eigen::MatrixXd get_R() {return R_;}

  /**
   * @brief Get the process noise covariance matrix `Q`.
   *
   * @return Process noise covariance matrix `Q_` (n x n).
   */
  Eigen::MatrixXd get_Q() {return Q_;}

  /**
   * @brief Get the initial error covariance matrix `P0`.
   *
   * @return Initial error covariance matrix `P0_` (n x n).
   */
  Eigen::MatrixXd get_P0() {return P0_;}

  /**
   * @brief Get the type of the state observer.
   *
   * @return A string representing the type of the state observer.
   */
  std::string get_type() const override;

  using SharedPtr = std::shared_ptr<KalmanFilterParam>;

protected:
  Eigen::MatrixXd R_, Q_, P0_;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS_PARAM__KALMAN_FILTER_PARAM_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  state_observer::KalmanFilterParam,
  state_observer::StateObserverParam)
