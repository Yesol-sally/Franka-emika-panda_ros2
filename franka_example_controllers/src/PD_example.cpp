#include <franka_example_controllers/PD_example.hpp>
#include <franka_example_controllers/robot_utils.hpp> // 이 파일은 로봇 관련 유틸리티 함수들을 포함하고 있습니다.

#include <cassert>
#include <cmath>
#include <exception>
#include <string> 

#include <Eigen/Eigen>

namespace franka_example_controllers { //hpp의 네임스페이스와 동일

controller_interface::InterfaceConfiguration //ROS2의 컨트롤러 인터페이스에서 사용되는 구성 정보를 나타냅니다.
PDExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
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
  }
  return config;
}

//변경부분
controller_interface::return_type PDExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // 1) 초기화: 기준 위치와 이전 오차 저장
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
      prev_error_.at(i) = 0.0;
    }
    initialization_flag_ = false;
  }

  // 2) PD 제어 계산
  double dt = period.seconds();  // 주기(초)
  for (int i = 0; i < num_joints; ++i) {
    double q        = state_interfaces_[i].get_value();
    double error    = initial_q_.at(i) - q;
    double derivative = (error - prev_error_.at(i)) / dt;
    double command  = q + Kp * error + Kd * derivative;

    command_interfaces_[i].set_value(command);
    prev_error_.at(i) = error;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn PDExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PDExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

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
  initialization_flag_ = true;
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PDExampleController,
                       controller_interface::ControllerInterface)
