
.. _program_listing_file_include_state_observers_luenberger.hpp:

Program Listing for File luenberger.hpp
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_state_observers_luenberger.hpp>` (``include/state_observers/luenberger.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   class Luenberger : public StateObserver
   {
   public:
     Luenberger(
       const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
       const Eigen::MatrixXd & C, const Eigen::MatrixXd & D,
       const Eigen::VectorXd & initial_state,
       const Eigen::MatrixXd & L);
   
     Luenberger(
       const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
       const Eigen::MatrixXd & C,
       const Eigen::VectorXd & initial_state,
       const Eigen::MatrixXd & L);
   
     Luenberger(
       const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
       const Eigen::MatrixXd & C,
       const Eigen::MatrixXd & L);
   
     Luenberger() {}
   
     ~Luenberger() {}
   
     void set_parameters(const StateObserverParam::SharedPtr state_observer_params) override;
   
     void set_observer_gain(const Eigen::MatrixXd & L);
   
     Eigen::MatrixXd update(const Eigen::VectorXd & measurement) override;
   
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
