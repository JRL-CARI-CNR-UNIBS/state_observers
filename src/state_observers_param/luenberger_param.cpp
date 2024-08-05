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

#include "state_observers_param/luenberger_param.hpp"

namespace state_observer
{

void
LuenbergerParam::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  StateObserverParam::initialize(node);

  StateObserverParam::fully_declare_parameter(
    "Luenberger_gain", rclcpp::PARAMETER_DOUBLE_ARRAY,
    "Luenberger gain matrix (L).");

  std::vector<double> flat_L;
  if (!node_->get_parameter("Luenberger_gain", flat_L)) {
    throw std::runtime_error("Failed to get Luenberger_gain parameter.");
  }

  L_ = Eigen::Map<Eigen::MatrixXd>(flat_L.data(), A_.rows(), C_.rows());
}


}  // namespace state_observer
