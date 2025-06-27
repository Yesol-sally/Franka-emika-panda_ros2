#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <memory>
#include <string>

namespace franka_example_controllers {

class KDLModelParam {
public:
  KDLModelParam(const std::string& urdf_string, const std::string& root, const std::string& tip);
  bool isValid() const;

  bool computeDynamics(
    const KDL::JntArray& q,
    const KDL::JntArray& dq,
    KDL::JntSpaceInertiaMatrix& mass,
    KDL::JntArray& coriolis,
    KDL::JntArray& gravity);

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  bool valid_;
};

} // namespace franka_example_controllers
