//
// Created by rgrandia on 08.07.19.
//

#ifndef OCS2_CTRL_ENDEFFECTORPOSITIONCONSTRAINT_H
#define OCS2_CTRL_ENDEFFECTORPOSITIONCONSTRAINT_H

#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>
#include <ocs2_core/constraint/ConstraintTerm.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

struct EndEffectorPositionConstraintSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Desired end effector position specified as h(x,u) = A*p(x,u) + b
  Eigen::MatrixXd Ab;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
class EndEffectorPositionConstraint final : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
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

 private:
  static constexpr size_t domain_dim_ = 1 + STATE_DIM + INPUT_DIM;
  static constexpr size_t range_dim_ = 3;

  using ad_interface_t = ocs2::CppAdCodeGenInterface<domain_dim_, range_dim_, scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_state_vector_t = typename Eigen::Matrix<ad_scalar_t, STATE_DIM, 1>;
  using ad_input_vector_t = typename Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1>;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using ad_dynamic_matrix_t = typename ad_interface_t::ad_dynamic_matrix_t;
  using ad_funtion_t = typename ad_interface_t::ad_funtion_t;
  using ad_fun_t = typename ad_interface_t::ad_fun_t;
  using domain_vector_t = typename ad_interface_t::domain_vector_t;
  using domain_matrix_t = typename ad_interface_t::domain_matrix_t;
  using variable_vector_t = typename ad_interface_t::variable_vector_t;
  using variable_matrix_t = typename ad_interface_t::variable_matrix_t;
  using range_vector_t = typename ad_interface_t::range_vector_t;
  using domain_range_matrix_t = typename ad_interface_t::domain_range_matrix_t;
  using range_domain_matrix_t = typename ad_interface_t::range_domain_matrix_t;
  using range_state_matrix_t = Eigen::Matrix<double, range_dim_, STATE_DIM>;
  using range_input_matrix_t = Eigen::Matrix<double, range_dim_, INPUT_DIM>;
  using domain_matrix_array_t = std::vector<domain_matrix_t, Eigen::aligned_allocator<domain_matrix_t>>;

 public:
  using ad_com_model_t = ComModelBase<12, ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<12, ad_scalar_t>;

  explicit EndEffectorPositionConstraint(int legNumber, EndEffectorPositionConstraintSettings settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels)
      : BASE(ocs2::ConstraintOrder::Linear),
        legNumber_(legNumber),
        settings_(std::move(settings)),
        libName_("EEPositionConstraint_" + std::to_string(legNumber_)) {
    setAdInterface(adComModel, adKinematicsModel);
    if (generateModels){
      cppAdCodeGenClass_->createModels(libName_, "");
    } else {
      cppAdCodeGenClass_->loadModels(libName_);
    };
  }

  EndEffectorPositionConstraint(const EndEffectorPositionConstraint& rhs)
      : BASE(rhs),
        legNumber_(rhs.legNumber_),
        settings_(rhs.settings_),
        libName_(rhs.libName_),
        cppAdCodeGenClass_(rhs.cppAdCodeGenClass_->clone()) {
    cppAdCodeGenClass_->loadModels(libName_);
  }

  EndEffectorPositionConstraint* clone() const override {
    return new EndEffectorPositionConstraint(*this);
  }

  void configure(const EndEffectorPositionConstraintSettings& settings) { settings_ = settings; };

  size_t getNumConstraints(scalar_t time) const override { return settings_.Ab.rows(); };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    domain_vector_t tapedInput;
    tapedInput << time, state, input;

    // Compute constraints
    range_vector_t eePositionWorld;
    cppAdCodeGenClass_->getFunctionValue(tapedInput, eePositionWorld);

    // Homogeneous coordinates
    Eigen::Matrix<scalar_t, 4, 1> eePositionWorldHom;
    eePositionWorldHom << eePositionWorld, 1.0;

    // Change to std::vector
    scalar_array_t constraintValue;
    for (int i = 0; i < settings_.Ab.rows(); i++) {
      constraintValue.emplace_back(settings_.Ab.row(i) * eePositionWorldHom);
    }
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    domain_vector_t tapedInput;
    tapedInput << time, state, input;

    // Compute derivatives
    domain_range_matrix_t stateInputJacobian;
    stateInputJacobian.setZero();  // Important! ccpAD only fills the non-zero terms
    cppAdCodeGenClass_->getJacobian(tapedInput, stateInputJacobian);

    // Homogeneous coordinates
    range_state_matrix_t drange_dx = stateInputJacobian.block(1, 0, STATE_DIM, range_dim_).transpose();
    range_input_matrix_t drange_du = stateInputJacobian.block(1 + STATE_DIM, 0, INPUT_DIM, range_dim_).transpose();

    // Convert to output format
    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues = getValue(time, state, input);
    for (int i = 0; i < settings_.Ab.rows(); i++) {
      linearApproximation.derivativeState.emplace_back(settings_.Ab.template block<1, range_dim_>(i, 0) * drange_dx);
      linearApproximation.derivativeInput.emplace_back(settings_.Ab.template block<1, range_dim_>(i, 0) * drange_du);
    }
    return linearApproximation;
  }

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override {
    // Assemble input
    domain_vector_t tapedInput;
    tapedInput << time, state, input;

    domain_matrix_array_t stateInputHessians(3);
    for (int i = 0; i < 3; i++) {
      cppAdCodeGenClass_->getHessian(tapedInput, stateInputHessians[i], i);
    }

    // Compute hessians for all constraints: For the i-th constraint, sum over entries in A times corresponding Hessian
    domain_matrix_array_t stateInputWeightedHessians;
    for (int i = 0; i < settings_.Ab.rows(); i++) {
      stateInputWeightedHessians.emplace_back(settings_.Ab(i, 0) * stateInputHessians[0] +
                                              settings_.Ab(i, 1) * stateInputHessians[1] +
                                              settings_.Ab(i, 2) * stateInputHessians[2]);
    }

    // Convert to output format
    QuadraticApproximation_t quadraticApproximation;
    auto linearApproximation = getLinearApproximation(time, state, input);
    quadraticApproximation.constraintValues = std::move(linearApproximation.constraintValues);
    quadraticApproximation.derivativeState = std::move(linearApproximation.derivativeState);
    quadraticApproximation.derivativeInput = std::move(linearApproximation.derivativeInput);
    for (int i = 0; i < settings_.Ab.rows(); i++) {
      quadraticApproximation.secondDerivativesState.emplace_back(stateInputWeightedHessians[i].block(1, 1, STATE_DIM, STATE_DIM));
      quadraticApproximation.secondDerivativesInput.emplace_back(stateInputWeightedHessians[i].block(1 + STATE_DIM, 1 + STATE_DIM, INPUT_DIM, INPUT_DIM));
      quadraticApproximation.derivativesInputState.emplace_back(stateInputWeightedHessians[i].block(1 + STATE_DIM, 1, INPUT_DIM, STATE_DIM));
    }
    return quadraticApproximation;
  }

 private:
  void adFootPosition(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                      ad_dynamic_vector_t& o_footPosition) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    ad_state_vector_t x = tapedInput.segment(1, STATE_DIM);
    ad_input_vector_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    using Vector3Ad = typename ad_kinematic_model_t::vector3d_t;
    using Matrix3Ad = typename ad_kinematic_model_t::matrix3d_t;
    using ad_joint_coordinate_t = typename ad_kinematic_model_t::joint_coordinate_t;
    using ad_base_coordinate_t = typename ad_kinematic_model_t::base_coordinate_t;
    Vector3Ad baseEulerAngles = x.segment(0, 3);
    Vector3Ad o_comPosition = x.segment(3, 3);  // in origin frame
    ad_joint_coordinate_t qJoints = x.segment(12, 12);

    // base coordinates [EulerAngles, base position in world]
    Vector3Ad com_base2CoM_ = adComModel.comPositionBaseFrame(qJoints);
    Matrix3Ad o_R_b_ = RotationMatrixBasetoOrigin<ad_scalar_t>(baseEulerAngles);
    ad_base_coordinate_t basePose;
    basePose << baseEulerAngles, o_comPosition - o_R_b_ * com_base2CoM_;

    // update kinematic model
    adKinematicsModel.update(basePose, qJoints);

    // Get foot position and Jacobian
    Vector3Ad footPositionInWorld;
    adKinematicsModel.footPositionOriginFrame(legNumber_, footPositionInWorld);
    o_footPosition = footPositionInWorld;
  }

  void setAdInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel) {
    ad_funtion_t adfunc_ = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      this->adFootPosition(adComModel, adKinematicsModel, x, y);
    };

    // Set Sparsity
    range_domain_matrix_t sparsityPattern;
    sparsityPattern.setOnes();
    // TODO (Ruben): Setting the sparsity pattern results in wrong derivatives for outputs that are not dependent on the inputs.
    //    sparsityPattern.col(0).setZero(); // Do not take derivative w.r.t time
    //    sparsityPattern.rightCols(parameter_dim_).setZero(); // Do not take derivative w.r.t parameters

    // Create models
    cppAdCodeGenClass_.reset(new ad_interface_t(adfunc_, sparsityPattern));
    cppAdCodeGenClass_->computeForwardModel(true);
    cppAdCodeGenClass_->computeJacobianModel(true);
    cppAdCodeGenClass_->computeHessianModel(true);
  };

  int legNumber_;
  EndEffectorPositionConstraintSettings settings_;
  std::string libName_;
  std::unique_ptr<ad_interface_t> cppAdCodeGenClass_;
};
}  // namespace switched_model

#endif  // OCS2_CTRL_ENDEFFECTORPOSITIONCONSTRAINT_H
