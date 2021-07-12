#pragma once

#include <Eigen/Core>
#include <memory>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

struct EndEffectorConstraintSettings {
  // Constraints are described by A() * v_base + b()
  Eigen::VectorXd b;
  Eigen::MatrixXd A;

  EndEffectorConstraintSettings() = default;

  EndEffectorConstraintSettings(size_t rows, size_t cols);

  void resize(size_t rows, size_t cols);
};

class EndEffectorConstraint : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Settings_t = EndEffectorConstraintSettings;

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;
  using adfunc_t = void (*)(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber, const ocs2::ad_vector_t& x,
                            ocs2::ad_vector_t& y);

  EndEffectorConstraint(const std::string& eeConstraintName, int legNumber, Settings_t settings, ad_com_model_t& adComModel,
                        ad_kinematic_model_t& adKinematicsModel, adfunc_t adfunc, bool generateModels);

  ~EndEffectorConstraint() override = default;

  EndEffectorConstraint* clone() const override;

  void configure(const Settings_t& settings);

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const override;

 protected:
  EndEffectorConstraint(const EndEffectorConstraint& rhs);

 private:
  Settings_t settings_;
  const int legStartIdx_;
  std::unique_ptr<ocs2::CppAdInterface> adInterface_;
};

}  // namespace switched_model
