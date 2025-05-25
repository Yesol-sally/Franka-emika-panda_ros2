// Copyright (c) 2023 Franka Robotics GmbH
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

#include <franka_example_controllers/PD_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
PDExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    // config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
PDExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type PDExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // 1) 현재 상태(q_, dq_) 갱신
  updateJointStates();

  // 2) 원래 초기 위치 기반 목표 궤적 계산 (q_goal)
  Vector7d q_goal = initial_q_;
  elapsed_time_ += period.seconds();
  double delta = M_PI/8.0 * (1 - std::cos(M_PI/2.5 * elapsed_time_));
  q_goal(3) += delta;
  q_goal(4) += delta;

  // 3) 속도 저역통과 필터 (노이즈 억제)
  const double alpha = 0.99;
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq_;

  // 4) 적분 동작
  Vector7d error = q_goal - q_;                             // 현재 위치 오차
  error_integral_ += error * period.seconds();             // 오차를 시간으로 적분

  // 5) **PD 위치 제어**: 
  //    pos_command = q_goal + Kp*(q_goal - q_) + Kd*( − dq_filtered_ ) + Ki*(error_integral_)
  Vector7d pos_command =
      q_goal
      + k_gains_.cwiseProduct(q_goal - q_)                 // P term
      + i_gains_.cwiseProduct(error_integral_)             // I term
      + d_gains_.cwiseProduct(-dq_filtered_);              // D term

  // 6) position 명령으로 내보내기
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(pos_command(i));
  }
  return controller_interface::return_type::OK;
}


CallbackReturn PDExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("i_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PDExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto i_gains = get_node()->get_parameter("i_gains").as_double_array();  
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (i_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "i_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (i_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "i_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }  
  for (int i = 0; i < num_joints; ++i) {
    i_gains_(i) = i_gains.at(i);    
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();
  error_integral_.setZero();    // 적분 누적 초기값

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PDExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  error_integral_.setZero();    // 적분 누적 리셋
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

void PDExampleController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PDExampleController,
                       controller_interface::ControllerInterface)