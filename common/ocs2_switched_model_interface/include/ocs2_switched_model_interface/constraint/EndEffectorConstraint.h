#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <Eigen/Core>
#include <memory>

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

class EndEffectorConstraint : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
  static constexpr size_t domain_dim_ = 1 + STATE_DIM + INPUT_DIM;
  static constexpr size_t range_dim_ = 3;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using Settings_t = EndEffectorConstraintSettings;

  using ad_interface_t = ocs2::CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using dynamic_vector_t = typename ad_interface_t::dynamic_vector_t;

  using constraint_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, -1, domain_dim_>;
  using timeStateInput_matrix_t = Eigen::Matrix<scalar_t, domain_dim_, domain_dim_>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;
  using adfunc_t = void (*)(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                            const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y);

  EndEffectorConstraint(ocs2::ConstraintOrder constraintOrder, std::string eeConstraintName, int legNumber, Settings_t settings,
                        ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, adfunc_t adfunc, bool generateModels,
                        bool loadModels = true);

  ~EndEffectorConstraint() = default;

  //! Note: Since the constructors are based on a copy we do not regenerate/generate the models
  EndEffectorConstraint(const EndEffectorConstraint& rhs);

  //! Note: Since the constructors are based on a copy we do not or regenerate/generate the models
  EndEffectorConstraint* clone() const override;

  void configure(const Settings_t& settings);

  size_t getNumConstraints(scalar_t time) const override;

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override;
  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override;
  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override;

 protected:
  Settings_t settings_;
  int legNumber_;

 private:
  void initAdModels(ocs2::ConstraintOrder constraintOrder, bool generateModels, bool loadModels, bool verbose = true);

  std::string libName_;
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> adInterface_;
};

}  // namespace switched_model
