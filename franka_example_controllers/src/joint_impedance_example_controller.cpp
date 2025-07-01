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

#include <franka_example_controllers/joint_impedance_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "franka_example_controllers/kdl_model_param.hpp" //추가

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();
  Vector7d q_goal = initial_q_;
  elapsed_time_ = elapsed_time_ + period.seconds();

  //############### q_, dq_ 출력 ##################
  std::cout << "q_: " << q_.transpose() << std::endl;
  std::cout << "dq_: " << dq_.transpose() << std::endl;
  //###############################################


  //############################################### 
  // KDL 기반 모델 파라미터 계산
  KDL::JntArray q_kdl(num_joints), dq_kdl(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    q_kdl(i) = q_(i);
    dq_kdl(i) = dq_(i);
  }
  KDL::JntSpaceInertiaMatrix mass(num_joints);
  KDL::JntArray coriolis(num_joints), gravity(num_joints);
  //출력
  if (kdl_model_param_ && kdl_model_param_->computeDynamics(q_kdl, dq_kdl, mass, coriolis, gravity)) {
    // std::cout << "Mass matrix diagonal: ";
    // for (int i = 0; i < num_joints; ++i) std::cout << mass(i, i) << " ";
    // std::cout << std::endl;
    std::cout << "Mass matrix: ";
    for (int i = 0; i < num_joints; ++i) {
    for (int j = 0; j < num_joints; ++j) {
      std::cout << mass(i, j) << " ";
    }
    std::cout << std::endl;
  }
    std::cout << "Coriolis: ";
    for (int i = 0; i < num_joints; ++i) std::cout << coriolis(i) << " ";
    std::cout << std::endl;
    std::cout << "Gravity: ";
    for (int i = 0; i < num_joints; ++i) std::cout << gravity(i) << " ";
    std::cout << std::endl;
  }
  // 자코비안 계산 (6x7 행렬)
  KDL::Jacobian jacobian(num_joints);
  if (kdl_model_param_ && kdl_model_param_->computeJacobian(q_kdl, jacobian)) {
    std::cout << "\nJacobian matrix (6x7):" << std::endl;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < num_joints; ++j) {
        std::cout << jacobian(i, j) << "\t";
      }
      std::cout << std::endl;
    }
  //FK 계산
    KDL::Frame end_effector_pose;
  if (kdl_model_param_ && kdl_model_param_->computeForwardKinematics(q_kdl, end_effector_pose)) {
    std::cout << "\n=== Forward Kinematics Results ===" << std::endl;
    
    // 엔드이펙터 위치 출력 (x, y, z)
    std::cout << "End-effector position (x, y, z): "
              << end_effector_pose.p[0] << ", "
              << end_effector_pose.p[1] << ", "
              << end_effector_pose.p[2] << std::endl;
    
    // 엔드이펙터 자세 출력 (Roll, Pitch, Yaw)
    double roll, pitch, yaw;
    end_effector_pose.M.GetRPY(roll, pitch, yaw);
    std::cout << "End-effector orientation (RPY): "
              << roll << ", " << pitch << ", " << yaw << std::endl;
    
    // 회전 행렬 출력 (3x3)
    std::cout << "End-effector rotation matrix (3x3):" << std::endl;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::cout << end_effector_pose.M(i, j) << "\t";
      }
      std::cout << std::endl;
    }
  } else {
    std::cout << "Forward kinematics computation failed!" << std::endl;
  }
  }
  //###############################################

  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));
  q_goal(3) += delta_angle;
  q_goal(4) += delta_angle;

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
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
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

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

//####################KDL 기반 로봇 동역학 모델 객체 초기화####################
  kdl_model_param_ = std::make_unique<KDLModelParam>(robot_description_, "fr3_link0", "fr3_link7");
  if (!kdl_model_param_->isValid()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize KDL model param.");
    return CallbackReturn::FAILURE;
  }
//########################################################################

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

void JointImpedanceExampleController::updateJointStates() {
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
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerInterface)
