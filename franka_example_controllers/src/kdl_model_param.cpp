#include "franka_example_controllers/kdl_model_param.hpp"
#include <kdl_parser/kdl_parser.hpp>


namespace franka_example_controllers {

KDLModelParam::KDLModelParam(const std::string& urdf_string, const std::string& root, const std::string& tip) : valid_(false) {
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) return;
  if (!tree.getChain(root, tip, chain_)) return;
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0, 0, -9.81));
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

  valid_ = true;
}

bool KDLModelParam::isValid() const { return valid_; }

bool KDLModelParam::computeDynamics(
    const KDL::JntArray& q,
    const KDL::JntArray& dq,
    KDL::JntSpaceInertiaMatrix& mass,
    KDL::JntArray& coriolis,
    KDL::JntArray& gravity) {
  if (!valid_) return false;
  if (dyn_param_->JntToMass(q, mass) < 0) return false;
  if (dyn_param_->JntToCoriolis(q, dq, coriolis) < 0) return false;
  if (dyn_param_->JntToGravity(q, gravity) < 0) return false;
  return true;
}
// 자코비안 계산 (6x7 행렬)
bool KDLModelParam::computeJacobian(const KDL::JntArray& q, KDL::Jacobian& jacobian) {
  if (!valid_) return false;
  return jac_solver_->JntToJac(q, jacobian) >= 0;
}
//FK 추가
bool KDLModelParam::computeForwardKinematics(const KDL::JntArray& q, KDL::Frame& end_effector_pose) {
  if (!valid_) return false;
  return fk_solver_->JntToCart(q, end_effector_pose) >= 0;
}

} // namespace franka_example_controllers
 