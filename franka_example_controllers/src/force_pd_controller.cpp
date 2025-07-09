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

#include <franka_example_controllers/force_pd_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
#include <kdl/jntarray.hpp> //초기 조인트값 정의를 위한
#include <kdl/frames.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>  // Publishers for bag recording
#include <std_msgs/msg/float64_multi_array.hpp>  // Publishers for bag recording
#include <rclcpp/rclcpp.hpp>  //시뮬레이션 중단 기능
#include <Eigen/Geometry>  //회정행렬 만들기

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
ForcePDController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ForcePDController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // config.names = franka_cartesian_pose_->get_state_interface_names();

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::return_type ForcePDController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();
  // Vector7d q_goal = initial_q_;
  elapsed_time_ = elapsed_time_ + period.seconds();

  // if (initialization_flag_) {
  //   // Get initial orientation and translation
  //   std::tie(orientation_, position_) =
  //       franka_cartesian_pose_->getCurrentOrientationAndTranslation();

    // initialization_flag_ = false;
  // }

  // printCartesianStates();    

  // double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));
  // q_goal(0) += delta_angle;
  // q_goal(4) += delta_angle;

  // const double kAlpha = 0.99;
  // dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  // Vector7d tau_d_calculated =
  //     k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  
  //###########################################
  //############### q_, dq_ 출력 ##################
  // std::cout << "q_: " << q_.transpose() << std::endl;
  // std::cout << "dq_: " << dq_.transpose() << std::endl;
  //###############################################
  getKDLmodel();
  KDL::JntArray q_kdl(num_joints);
  KDL::JntArray q_kdl_dot(num_joints);
  
    for (int i = 0; i < num_joints; ++i) {
      q_kdl(i) = q_(i);
      q_kdl_dot(i) = dq_(i);

    }
  // joint 상태 읽은 뒤(= q_ 채운 뒤) 바로 추가
  if (fk_pos_solver_) {

    fk_pos_solver_->JntToCart(q_kdl, ee_frame_);

    // std::cout << "EE pos = ["    // 위치
    //           << ee_frame_.p.x() << ", "
    //           << ee_frame_.p.y() << ", "
    //           << ee_frame_.p.z() << "]  ";
// std::cout << "EE Frame:\n"
//           << "(0,0) = " << ee_frame_.M(0,0) << "  "
//           << "(0,1) = " << ee_frame_.M(0,1) << "  "
//           << "(0,2) = " << ee_frame_.M(0,2) << "\n"
//           << "(1,0) = " << ee_frame_.M(1,0) << "  "
//           << "(1,1) = " << ee_frame_.M(1,1) << "  "
//           << "(1,2) = " << ee_frame_.M(1,2) << "\n"
//           << "(2,0) = " << ee_frame_.M(2,0) << "  "
//           << "(2,1) = " << ee_frame_.M(2,1) << "  "
//           << "(2,2) = " << ee_frame_.M(2,2) << std::endl;

// 이부분은 왜 넣은거지
    tf2::quaternionKDLToEigen(ee_frame_.M, ee_ori_quat_);
    ee_ori_rot_ = ee_ori_quat_.toRotationMatrix();

    // ee_ori_eulerZYZ_ = Eigen::EulerAngles<double, Eigen::EulerSystemZYZ>(ee_ori_rot_);
    // euler_phi  = ee_ori_eulerZYZ_.alpha() * 180.0 / M_PI;  // X축
    // euler_theta = ee_ori_eulerZYZ_.beta() * 180.0 / M_PI;   // Y축
    // euler_psi   = ee_ori_eulerZYZ_.gamma() * 180.0 / M_PI;  // Z축

    // Eigen::Matrix3d R0;
    // R0 << ee_frame_.M(0,0), ee_frame_.M(0,1), ee_frame_.M(0,2),
    //       ee_frame_.M(1,0), ee_frame_.M(1,1), ee_frame_.M(1,2),
    //       ee_frame_.M(2,0), ee_frame_.M(2,1), ee_frame_.M(2,2);

    // // 3a) Eigen의 eulerAngles(2,1,0) 사용  ──────────────────────────────
    // Eigen::Vector3d eulerZYX = R0.eulerAngles(2, 1, 0);  // [Z, Y, X] 순
    // euler_phi = eulerZYX[0];
    // euler_theta = eulerZYX[1];
    // euler_psi = eulerZYX[2];

    // std::cout << "Eulerangle" << eulerZYX[0] << ", "
    // << eulerZYX[1] << ", " << eulerZYX[2] << std::endl;
    // ee_ori_eulerZYX_ = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>(R0);

  //Euler angle(ZYX)로 변환
    ee_ori_eulerZYX_ = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>(ee_ori_rot_);
    // euler_phi  = ee_ori_eulerZYX_.alpha();   // Z축
    // euler_theta = ee_ori_eulerZYX_.beta();   // Y축
    // euler_psi   = ee_ori_eulerZYX_.gamma();  // X축

    // euler_phi   = wrapToPi(euler_phi);
    // euler_theta   = wrapToPi(euler_theta);
    // euler_psi   = wrapToPi(euler_psi);

    Eigen::Vector3d eul_cur;
    eul_cur << ee_ori_eulerZYX_.alpha(),   // ψ (Z-yaw)
                ee_ori_eulerZYX_.beta(),   // θ (Y-pitch)
                ee_ori_eulerZYX_.gamma();  // φ (X-roll)

    // ── ② unwrap으로 연속성 확보 ──────────────────────────
    eul_cur = unwrapEulerZYX(eul_cur, eul_prev_);   // ❷ 점프 제거

    // ── ③ 필요 시 ±π 로만 시각화 (제어에는 unwrap 값 사용) ─
    euler_phi   = eul_cur.z();               // 또는 eul_cur(2)
    euler_theta = eul_cur.y();
    euler_psi   = eul_cur.x();

    // std::cout << "Eulerangle" << ee_ori_eulerZYX_ << std::endl;
    // std::cout << "Recovered Euler (roll, pitch, yaw): "
    //           << euler_phi << ", " << euler_theta << ", " << euler_psi << std::endl;
  
// Rotation matrix 기반으로 오차 생성
  // Eigen::Matrix3d R_ref_ = makeRotationZYX(30.0, -15.0, 10.0);
  // ee_ori_rot_err = R_ref_.transpose()*ee_ori_rot_;

  }

  //###########################################
  


  //###########################################
  KDL::JntArray q_initial_kdl(7);
  for (std::size_t i = 0; i < 7; ++i) {
    q_initial_kdl(i) = initial_q_(i);
  }
  KDL::Frame ee_initial_frame_;
  
  fk_pos_solver_->JntToCart(q_initial_kdl, ee_initial_frame_);

  Vector6d cart_pos_goal;
  Vector6d cart_pos_err;
  Vector6d cart_vel_goal = Vector6d::Zero();
  Vector6d cart_vel_current;
  Vector6d cart_initial;

  // cart_pos_goal(0) = 0.303891;
  // cart_pos_goal(1) = 0.007245;
  // cart_pos_goal(2) = 0.651902;
  // cart_pos_goal(3) = -2.0504738;
  // cart_pos_goal(4) = 2.715549;
  // cart_pos_goal(5) = 1.790270;


  cart_initial(0) = ee_initial_frame_.p.x();
  cart_initial(1) = ee_initial_frame_.p.y();
  cart_initial(2) = ee_initial_frame_.p.z();
  // cart_pos_goal(0) = 0.1;
  // cart_pos_goal(1) = 0;
  // cart_pos_goal(2) = 0;

  // // ZYZ 오일러 각 추출
  double z, y, x; //z1, y, z2
  // ee_initial_frame_.M.GetEulerZYZ(z1, y, z2); 
  ee_initial_frame_.M.GetEulerZYX(z, y, x);
  cart_initial(3) = z - 2;
  cart_initial(4) = y;
  cart_initial(5) = x;
  
  // Eigen::Matrix3d R1;
  //   R1 << ee_initial_frame_.M(0,0), ee_initial_frame_.M(0,1), ee_initial_frame_.M(0,2),
  //         ee_initial_frame_.M(1,0), ee_initial_frame_.M(1,1), ee_initial_frame_.M(1,2),
  //         ee_initial_frame_.M(2,0), ee_initial_frame_.M(2,1), ee_initial_frame_.M(2,2);

  //   // 3a) Eigen의 eulerAngles(2,1,0) 사용  ──────────────────────────────
  // Eigen::Vector3d eulerZYX = R1.eulerAngles(2, 1, 0);  // [Z, Y, X] 순
  // cart_initial(3) = eulerZYX[0];
  // cart_initial(4) = eulerZYX[1];
  // cart_initial(5) = eulerZYX[2];


  // std::cout << "cart_initial" << "\n"<< cart_initial << std::endl;

  double delta_diff = 0.5 * std::cos(M_PI /3 * elapsed_time_);
  // double delta_diff2 = 0.2 * std::cos(M_PI /5 * elapsed_time_);
  cart_initial(2) += delta_diff;
  // cart_initial(4) += delta_diff2;
  Eigen::Vector3d eul_cur2;
  eul_cur2 << cart_initial(3), cart_initial(4), cart_initial(5);
  eul_cur2 = unwrapEulerZYX(eul_cur2, eul_prev_2);
  cart_initial(3) = eul_cur2.z();
  cart_initial(4) = eul_cur2.y();
  cart_initial(5) = eul_cur2.x();

  cart_pos_goal(0) = cart_initial(0);
  cart_pos_goal(1) = cart_initial(1);
  cart_pos_goal(2) = cart_initial(2);
  cart_pos_goal(3) = cart_initial(3);
  cart_pos_goal(4) = cart_initial(4);
  cart_pos_goal(5) = cart_initial(5);
  

  // cart_pos_goal(0) = 0.303891;
  // cart_pos_goal(1) = 0.007245;
  // cart_pos_goal(2) = 0.651902;
  // cart_pos_goal(3) = -2.0504738;
  // cart_pos_goal(4) = 2.715549;
  // cart_pos_goal(5) = 1.790270;

  cart_pos_err(0) = cart_pos_goal(0) - ee_frame_.p.x();
  cart_pos_err(1) = cart_pos_goal(1) - ee_frame_.p.y();
  cart_pos_err(2) = cart_pos_goal(2) - ee_frame_.p.z();
  // cart_pos_err(3) = cart_pos_goal(3) - euler_phi;
  // cart_pos_err(4) = cart_pos_goal(4) - euler_theta;
  // cart_pos_err(5) = cart_pos_goal(5) - euler_psi;
  cart_pos_err(3) = cart_pos_goal(3) - euler_phi;
  cart_pos_err(4) = cart_pos_goal(4) - euler_theta;
  cart_pos_err(5) = cart_pos_goal(5) - euler_psi;

  Eigen::Vector3d eul_cur3;
  eul_cur3 << cart_pos_err(3), cart_pos_err(4), cart_pos_err(5);
  eul_cur3 = unwrapEulerZYX(eul_cur3, eul_prev_3);
  cart_pos_err(3) = eul_cur3.z();
  cart_pos_err(4) = eul_cur3.y();
  cart_pos_err(5) = eul_cur3.x();
  // cart_pos_err(3)   = wrapToPi(cart_pos_err(3));
  // cart_pos_err(4)   = wrapToPi(cart_pos_err(4));
  // cart_pos_err(5)   = wrapToPi(cart_pos_err(5));


// std::cout << "=======cart_pos_error=======\n"
//           << "error z: " << cart_pos_err(0) << "\n"
//           << "error y: " << cart_pos_err(1) << "\n"
//           << "error x: " << cart_pos_err(2) 
//           << std::endl;

  Eigen::Matrix<double, 6,7> jac_ana_eigen = getCrtAnalyticJacobian(q_kdl);
  cart_vel_current = jac_ana_eigen * dq_;
  // std::cout << "k_gains : " << k_gains_ << std::endl;
  // std::cout << "cart_pos_err" << "\n"<< cart_pos_err << std::endl;  // 출력!
  Vector6d force_calculated = k_gains_.cwiseProduct(cart_pos_err) + d_gains_.cwiseProduct(cart_vel_goal - cart_vel_current);
  // std::cout << "force_calculated" << force_calculated << std::endl;
  Vector7d tau_d_calculated = jac_ana_eigen.transpose() * force_calculated;
  // Vector7d tau_total = tau_d_calculated;
//#################null space control 추가
  Eigen::Matrix<double, 7, 1> K0;
  K0.setConstant(5);
  Vector7d tau_n = Nullspace_controller(mass_matrix_, dq_, jac_ana_eigen, K0);
  Vector7d tau_total = tau_d_calculated; // + tau_n;
//#################중력보상 추가
  // Eigen::Matrix<double, 7, 1> gravity_eigen;
  // for (int i = 0; i < num_joints; ++i) {
  //   gravity_eigen(i) = gravity_(i);
  // }
  // tau_total = tau_total + gravity_eigen;
  // tau_total = gravity_eigen; //중력보상으로만 동작해보기
//#################중력보상 추가

//######자코비안 미분
  // Eigen::Matrix<double, 6,7> jacdot_ana_eigen = getCrtAnalyticJacobianDot(q_kdl, q_kdl_dot);
  // std::cout << "jacdot_ana_eigen" << "\n"<< jacdot_ana_eigen << std::endl;

  // std::cout << "tau_total" << "\n"<< tau_total << std::endl; // 출력!
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_total(i));
  }

  // === 1) ee_frame_.p 퍼블리시 ===
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = get_node()->now();
    msg.header.frame_id = "base_link";  // 필요에 따라 변경
    msg.point.x = ee_frame_.p.x();
    msg.point.y = ee_frame_.p.y();
    msg.point.z = ee_frame_.p.z();
    ee_position_pub_->publish(msg);
  }

  // === 2) EE orientation (roll-pitch-yaw, ZYX) publish ===
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label  = "rpy";
    msg.layout.dim[0].size   = 3;
    msg.layout.dim[0].stride = 3;

    msg.data = {euler_phi,   // Z-axis rotation (yaw)
                euler_theta, // Y-axis rotation (pitch)
                euler_psi};  // X-axis rotation (roll)

    ee_orientation_pub_->publish(msg);
  }

  // === 3) cart_pos_err 퍼블리시 ===
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "error6";
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 6;
    msg.data = std::vector<double>(cart_pos_err.data(), cart_pos_err.data() + 6);
    cart_pos_err_pub_->publish(msg);
  }

  // === 4) tau_total 퍼블리시 ===
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "tau7";
    msg.layout.dim[0].size = 7;
    msg.layout.dim[0].stride = 7;
    msg.data = std::vector<double>(tau_total.data(), tau_total.data() + 7);
    tau_total_pub_->publish(msg);
  }

    // === 5) cart_goal 퍼블리시 ===
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "cart_goal";
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 6;
    msg.data = std::vector<double>(cart_pos_goal.data(), cart_pos_goal.data() + 6);
    cart_goal_pub_->publish(msg);
  }
    return controller_interface::return_type::OK;
}

CallbackReturn ForcePDController::on_init() {
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

CallbackReturn ForcePDController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(crt_dim)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 crt_dim, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(crt_dim)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 crt_dim, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < crt_dim; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
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

  // jdot_solver_ = std::make_shared<KDL::ChainJntToJacDotSolver>(kdl_model_param_->getChain());
  //########################################################################

  // 이미 robot_description_ 을 갖고 있으니 그대로 파싱
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description_, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL tree parse failed");
    return CallbackReturn::FAILURE;
  }

  // base-link ↔ tool-link 이름은 URDF에 맞게!
  if (!tree.getChain("fr3_link0", "fr3_link7", kdl_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Chain extraction failed");
    return CallbackReturn::FAILURE;
  }
  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  //#################################################
  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  // --- Publishers ---
  ee_position_pub_ = get_node()->create_publisher<geometry_msgs::msg::PointStamped>(
    "ee_position", rclcpp::SystemDefaultsQoS());
  ee_orientation_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "ee_orientation_rpy", rclcpp::SystemDefaultsQoS());
  cart_pos_err_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "cart_pos_err", rclcpp::SystemDefaultsQoS());
  tau_total_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "tau_total", rclcpp::SystemDefaultsQoS());
  cart_goal_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "cart_goal", rclcpp::SystemDefaultsQoS());    
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForcePDController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;
  // initialization_flag_ = true;


  // ZYX Euler 초기화 (= 첫 브랜치)
  Eigen::EulerAngles<double, Eigen::EulerSystemZYX> eul0(ee_ori_rot_);
  eul_prev_ = {eul0.alpha(), eul0.beta(), eul0.gamma()};
  eul_prev_2 = {eul0.alpha(), eul0.beta(), eul0.gamma()};
  eul_prev_3 = {eul0.alpha(), eul0.beta(), eul0.gamma()};

  //franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);


  return CallbackReturn::SUCCESS;
}

//added

// controller_interface::CallbackReturn JointImpedanceExampleController::on_deactivate(
//     const rclcpp_lifecycle::State& /*previous_state*/) {
//   franka_cartesian_pose_->release_interfaces();
//   return CallbackReturn::SUCCESS;
// }

//added


void ForcePDController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

 void ForcePDController::getKDLmodel(){
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
    // std::cout << "Coriolis: ";
    // for (int i = 0; i < num_joints; ++i) std::cout << coriolis(i) << " ";
    // std::cout << std::endl;
    // std::cout << "Gravity: ";
    // for (int i = 0; i < num_joints; ++i) std::cout << gravity(i) << " ";
    // std::cout << std::endl;
    mass_matrix_ = mass;
    gravity_ = gravity; // <-- 중력항 저장
  }
  //###############################################
 }


 Eigen::Matrix<double, 6,7> ForcePDController::getCrtAnalyticJacobian(KDL::JntArray q_in_form_of_kdl){
  // geonetric 자코비안 계산 (6x7 행렬)
    KDL::Jacobian jac_kdl(num_joints);
    kdl_model_param_->computeJacobian(q_in_form_of_kdl, jac_kdl);

    Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac_eigen(jac_kdl.data.data());
    // std::cout << "jac" << jac_eigen << std::endl;   
    
    // int jac_rank = matrixRank(jac_eigen);
    // std::cout << "Jac rank: " << jac_rank << std::endl;
    // if (jac_rank < 6){
    //       rclcpp::shutdown();
    // }



    //geometric <> analytic jacobian conversion
    sphi = std::sin(euler_phi),  cphi = std::cos(euler_phi);
    stheta = std::sin(euler_theta), ctheta = std::cos(euler_theta);

    // T_ZYZ << 0, -sphi,  cphi * stheta,
    //           0,  cphi,  sphi * stheta,
    //           1,     0,        ctheta;
    T_ZYX << 0,     -sphi,   cphi * ctheta, 
              0,     cphi,   sphi,
              1,     0,     -stheta;
    
    // T_A_ZYZ.bottomRightCorner<3,3>() = T_ZYZ;
    // Eigen::Matrix<double, 6,7> jac_ana_eigen = T_A_ZYZ.inverse() * jac_eigen; 
    //해더에서 Eigen::Matrix<double, 6, 6> T_A_ZYX = Eigen::Matrix<double, 6, 6>::Identity();으로 정의하고 있고, 오른쪽 아래부분을 덮어씌움
    T_A_ZYX.bottomRightCorner<3,3>() = T_ZYX;
    Eigen::Matrix<double, 6,7> jac_ana_eigen = T_A_ZYX.inverse() * jac_eigen; 
    //std::cout << "T_A_ZYZ:" << T_A_ZYZ << std::endl; //출력
    // std::cout << "jac_ana_eigen:" << jac_ana_eigen << std::endl;   
    // std::cout << "jac_ana" << jac_ana_eigen << std::endl;   
    // std::cout << "Jac_ana rank: " << matrixRank(jac_ana_eigen) << std::endl;
    return jac_ana_eigen;
  //###########################################
 }

// analytic Jacobian time-derivative
// Eigen::Matrix<double,6,7> ForcePDController::getCrtAnalyticJacobianDot(    
//     const KDL::JntArray& q,
//     const KDL::JntArray& qdot)
// {
//   const unsigned int n = q.rows(); // == num_joints


//   // 1) KDL::JntArrayVel 생성
//   KDL::JntArrayVel qvel(num_joints);
//   for (unsigned int i = 0; i < num_joints; ++i) {
//     qvel.q(i)     = q(i);
//     qvel.qdot(i)  = qdot(i);
//   }

//   // 2) geometric Jacobian derivative (6×7)
//   KDL::Jacobian jacdot_kdl(n);
//   int ret = jdot_solver_->JntToJacDot(qvel, jacdot_kdl);
//   if (ret < 0) {
//     return Eigen::Matrix<double,6,7>::Zero();
//   }
//   Eigen::Map<const Eigen::Matrix<double,6,7,Eigen::RowMajor>>
//     jacdot_eigen(jacdot_kdl.data.data());

  
//   //geometric <> analytic jacobian conversion
//   sphi   = std::sin(euler_phi);   cphi   = std::cos(euler_phi);
//   stheta = std::sin(euler_theta);   ctheta = std::cos(euler_theta);

//   // T_ZYX_ 구성
//   T_ZYX << 0,    -sphi,       cphi*ctheta,
//             0,     cphi,       sphi,
//             1,        0,         -stheta;
//   // analytic 변환 블록에 삽입
//   T_A_ZYX.bottomRightCorner<3,3>() = T_ZYX;
//   Eigen::Matrix<double,6,7> jacdot_ana_eigen = T_A_ZYX.inverse() * jacdot_eigen;

//   return jacdot_ana_eigen;
// }

//##########Null space controller#############
// Null-space controller implementation
Eigen::Matrix<double, 7, 1> ForcePDController::Nullspace_controller(
    const KDL::JntSpaceInertiaMatrix& mass,
    const Eigen::Matrix<double, 7, 1>& dq,
    const Eigen::Matrix<double, 6, 7>& jac_ana,
    const Eigen::Matrix<double, 7, 1>& K0) {
  // Convert KDL mass matrix to Eigen
  Eigen::Matrix<double, 7, 7> M;
    for (unsigned int i = 0; i < 7; ++i)
      for (unsigned int j = 0; j < 7; ++j)
        M(i, j) = mass(i, j);

  Eigen::Matrix<double, 7, 7> M_inv = M.inverse();

  // Operational space inertia
  Eigen::Matrix<double, 6, 6> lambda_inv = jac_ana * M_inv * jac_ana.transpose();
  Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
  Eigen::Matrix<double, 7, 6> Jbar = M_inv * jac_ana.transpose() * lambda;

  // Null-space projection
  Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - Jbar * jac_ana;
  Eigen::Matrix<double, 7, 1> tau_n = K0.asDiagonal() * N * N.transpose() * dq;
  return tau_n;
}
//##########Null space controller#############

// matrix rank
int ForcePDController::matrixRank(const Eigen::MatrixXd & M, double tol)
{
  Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
  if (tol < 0) tol = lu.threshold();
  return lu.rank();
}

double ForcePDController::wrapToPi(double rad)
{
 double two_pi = 2.0 * M_PI;
  while (rad >  M_PI)  rad -= two_pi;
  while (rad <= -M_PI) rad += two_pi;
  return rad;
}
// degree → radian 변환 함수
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

Eigen::Vector3d ForcePDController::unwrapEulerZYX(
    const Eigen::Vector3d& cur,
    Eigen::Vector3d&       prev) {

  Eigen::Vector3d out = cur;
  const double two_pi = 2.0 * M_PI;
  for (int i = 0; i < 3; ++i) {
    double diff = cur(i) - prev(i);
    if (diff >  M_PI) out(i) -= two_pi;
    if (diff < -M_PI) out(i) += two_pi;
  }
  prev = out;
  return out;
}


// Z→Y→X 순 회전행렬 생성 함수
// Eigen::Matrix3d makeRotationZYX(double z_deg, double y_deg, double x_deg)
// {
//     // degree → radian
//     double z_rad = deg2rad(z_deg);
//     double y_rad = deg2rad(y_deg);
//     double x_rad = deg2rad(x_deg);

//     // 각 축에 대한 AngleAxis 생성
//     Eigen::AngleAxisd rot_z(z_rad, Eigen::Vector3d::UnitZ());
//     Eigen::AngleAxisd rot_y(y_rad, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd rot_x(x_rad, Eigen::Vector3d::UnitX());

//     // Z축 회전 후 Y축, 그다음 X축 회전 순으로 곱함
//     Eigen::Quaterniond q = rot_z * rot_y * rot_x;

//     // Quaternion → 회전행렬
//     return q.toRotationMatrix();
// }

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForcePDController,
                       controller_interface::ControllerInterface)