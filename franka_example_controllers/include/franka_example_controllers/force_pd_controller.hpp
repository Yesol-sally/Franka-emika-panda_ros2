#pragma once

#include <string>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/loaned_state_interface.hpp"
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

//added
#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

#include <franka_example_controllers/kdl_model_param.hpp>
// joint_impedance_example_controller.hpp (또는 .cpp 상단)
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>  // FK-solver
#include <kdl/tree.hpp>                        // KDL::Tree
#include <kdl_parser/kdl_parser.hpp>           // kdl_parser::treeFromString
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/jacobian.hpp>
// #include <kdl/frames.hpp>

//added

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class ForcePDController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    // added 
  //CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  // added
  using Vector6d = Eigen::Matrix<double, 6, 1>;

 private:
  // //added
  // std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  // void printCartesianStates();
  // //added

  // Eigen::Quaterniond orientation_;
  // // Eigen::Vector3d position_;
  // const bool k_elbow_activated_{false};
  // bool initialization_flag_{true};

  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;
  const int crt_dim = 6;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector6d k_gains_;
  Vector6d d_gains_;
  
  double elapsed_time_{0.0};
  void updateJointStates();
  void getKDLmodel();
  Eigen::Matrix<double, 6,7> getCrtAnalyticJacobian(KDL::JntArray q_in_form_of_kdl);

  // Jacobian_dot
  Eigen::Matrix<double,6,7> getCrtAnalyticJacobianDot(KDL::JntArray q, KDL::JntArray qdot);
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jdot_solver_; 

   // KDL 모델 파라미터 계산 객체
  std::unique_ptr<KDLModelParam> kdl_model_param_; // 추가된 부분
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  KDL::Frame ee_frame_;        // 결과 저장용 (위치+자세)
  Eigen::Quaterniond ee_ori_quat_;
//   Eigen::EulerAngles<double, Eigen::EulerSystemZYZ> ee_ori_eulerZYZ_;
  Eigen::EulerAngles<double, Eigen::EulerSystemZYX> ee_ori_eulerZYX_;

  Eigen::Matrix3d ee_ori_rot_;

  double euler_phi {0.0};
  double euler_theta {0.0};  
  double euler_psi {0.0};  
  double sphi, cphi, stheta, ctheta;
//   Eigen::Matrix3d T_ZYZ;
//   Eigen::Matrix<double, 6, 6> T_A_ZYZ = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix3d T_ZYX;
  Eigen::Matrix<double, 6, 6> T_A_ZYX = Eigen::Matrix<double, 6, 6>::Identity();
  KDL::JntArray gravity_; 
  Vector7d gravity_matrix_;

  KDL::JntSpaceInertiaMatrix mass_matrix_; //null space에서 사용할 mass matrix 저장할 변수
  Eigen::Matrix<double, 7, 1> Nullspace_controller(
      const KDL::JntSpaceInertiaMatrix& mass,
      const Eigen::Matrix<double, 7, 1>& dq,
      const Eigen::Matrix<double, 6, 7>& jac_ana,
      const Eigen::Matrix<double, 7, 1>& K0);
};

}  // namespace franka_example_controllers