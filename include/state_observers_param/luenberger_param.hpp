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

#ifndef STATE_OBSERVERS_PARAM__LUENBERGER_PARAM_HPP_
#define STATE_OBSERVERS_PARAM__LUENBERGER_PARAM_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "state_observers_param/state_observer_param.hpp"


namespace state_observer
{

/**
 * @class LuenbergerParam
 * @brief Parameters for the Luenberger observer.
 *
 * The `LuenbergerParam` class extends `StateObserverParam` to include
 * the observer gain matrix specific to the Luenberger observer.
 */
class LuenbergerParam : public StateObserverParam
{
public:
  /**
   * @brief Default constructor.
   */
  LuenbergerParam() {}

  /**
   * @brief Destructor.
   */
  virtual ~LuenbergerParam() {}

  /**
   * @brief Initialize the parameters from a ROS 2 lifecycle node.
   *
   * This method reads the parameters from the provided node and initializes
   * the state-space matrices and observer gain matrix.
   *
   * @param node Shared pointer to an `rclcpp_lifecycle::LifecycleNode`.
   *
   * @throws std::runtime_error if parameter retrieval fails.
   */
  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Get the observer gain matrix `L`.
   *
   * @return Observer gain matrix `L_` (n x q).
   */
  Eigen::MatrixXd get_observer_gain() { return L_; }

  /**
   * @brief Get the type of the state observer.
   *
   * @return A string representing the type of the state observer.
   */
  std::string get_type() const override;
  
  using SharedPtr = std::shared_ptr<LuenbergerParam>;

protected:
  Eigen::MatrixXd L_;
};

}  // namespace state_observer

#endif  // STATE_OBSERVERS_PARAM__LUENBERGER_PARAM_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  state_observer::LuenbergerParam,
  state_observer::StateObserverParam)
