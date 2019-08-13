//
// Created by rgrandia on 30.06.19.
//

#ifndef OCS2_CTRL_ENDEFFECTORVELOCITYCONTRAINT_H
#define OCS2_CTRL_ENDEFFECTORVELOCITYCONTRAINT_H

#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>
#include <ocs2_core/constraint/ConstraintTerm.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

struct EndEffectorVelocityConstraintSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constraints are  A * v_world + b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
class EndEffectorVelocityConstraint final : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
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

  explicit EndEffectorVelocityConstraint(int legNumber, EndEffectorVelocityConstraintSettings settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels)
      : BASE(ocs2::ConstraintOrder::Linear), legNumber_(legNumber), settings_(std::move(settings)), libName_("EEVelocityConstraint_" + std::to_string(legNumber_)), libFolder_("/tmp/ocs2") {
    setAdInterface(adComModel, adKinematicsModel);
    if (generateModels){
      cppAdCodeGenClass_->createModels(libName_, libFolder_);
    } else {
      cppAdCodeGenClass_->loadModels(libName_, libFolder_);
    };
  }

  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint&rhs) :
  BASE(rhs),
  legNumber_(rhs.legNumber_),
  settings_(rhs.settings_),
  libName_(rhs.libName_),
  libFolder_(rhs.libFolder_),
  cppAdCodeGenClass_(rhs.cppAdCodeGenClass_->clone())
  {
    cppAdCodeGenClass_->loadModels(libName_, libFolder_);
  }

  EndEffectorVelocityConstraint* clone() const override {
    return new EndEffectorVelocityConstraint(*this);
  }

  void configure(const EndEffectorVelocityConstraintSettings& settings) { settings_ = settings; };

  size_t getNumConstraints(scalar_t time) const override { return settings_.A.rows(); };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    domain_vector_t tapedInput;
    tapedInput << time, state, input;

    // Compute constraints
    range_vector_t eeVelocityWorld;
    cppAdCodeGenClass_->getFunctionValue(tapedInput, eeVelocityWorld);

    // Change to std::vector
    scalar_array_t constraintValue;
    for (int i = 0; i < settings_.A.rows(); i++) {
      constraintValue.emplace_back( settings_.A.row(i) * eeVelocityWorld + settings_.b[i] );
    }
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    domain_vector_t tapedInput;
    tapedInput << time, state, input;

    // Compute end effector velocity and derivatives
    domain_range_matrix_t stateInputJacobian;
    stateInputJacobian.setZero();  // Important! ccpAD only fills the non-zero terms
    cppAdCodeGenClass_->getJacobian(tapedInput, stateInputJacobian);

    // Collect constraint terms
    range_state_matrix_t drange_dx = stateInputJacobian.block(1, 0, STATE_DIM, range_dim_).transpose();
    range_input_matrix_t drange_du = stateInputJacobian.block(1 + STATE_DIM, 0, INPUT_DIM, range_dim_).transpose();

    // Convert to output format
    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues = getValue(time, state, input);
    for (int i = 0; i < settings_.A.rows(); i++) {
      linearApproximation.derivativeState.emplace_back( settings_.A.row(i) * drange_dx );
      linearApproximation.derivativeInput.emplace_back( settings_.A.row(i) * drange_du );
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
    for (int i = 0; i < settings_.A.rows(); i++) {
      stateInputWeightedHessians.emplace_back(settings_.A(i, 0) * stateInputHessians[0] +
          settings_.A(i, 1) * stateInputHessians[1] +
          settings_.A(i, 2) * stateInputHessians[2]);
    }

    // Convert to output format
    QuadraticApproximation_t quadraticApproximation;
    auto linearApproximation = getLinearApproximation(time, state, input);
    quadraticApproximation.constraintValues = std::move(linearApproximation.constraintValues);
    quadraticApproximation.derivativeState = std::move(linearApproximation.derivativeState);
    quadraticApproximation.derivativeInput = std::move(linearApproximation.derivativeInput);
    for (int i = 0; i < settings_.A.rows(); i++) {
      quadraticApproximation.secondDerivativesState.emplace_back(stateInputWeightedHessians[i].block(1, 1, STATE_DIM, STATE_DIM));
      quadraticApproximation.secondDerivativesInput.emplace_back(stateInputWeightedHessians[i].block(1 + STATE_DIM, 1 + STATE_DIM, INPUT_DIM, INPUT_DIM));
      quadraticApproximation.derivativesInputState.emplace_back(stateInputWeightedHessians[i].block(1 + STATE_DIM, 1, INPUT_DIM, STATE_DIM));
    }
    return quadraticApproximation;
  }

 private:
  void adFootVelocity(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                    ad_dynamic_vector_t& o_footVelocity) {
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
    Vector3Ad o_comPosition = x.segment(3, 3);            // in origin frame
    Vector3Ad com_baseAngularVelocity = x.segment(6, 3);  // in com frame
    Vector3Ad com_comLinearVelocity = x.segment(9, 3);    // in com frame
    ad_joint_coordinate_t qJoints = x.segment(12, 12);
    ad_joint_coordinate_t dqJoints = u.segment(12, 12);

    // base coordinates [EulerAngles, base position in world]
    Vector3Ad com_base2CoM_ = adComModel.comPositionBaseFrame(qJoints);
    Matrix3Ad o_R_b_ = RotationMatrixBasetoOrigin<ad_scalar_t>(baseEulerAngles);
    ad_base_coordinate_t basePose;
    basePose << baseEulerAngles, o_comPosition - o_R_b_ * com_base2CoM_;

    // baseLocalVelocities_ = [omega, vel]_base in com frame
    Vector3Ad com_baseLinearVelocity = com_comLinearVelocity + com_base2CoM_.cross(com_baseAngularVelocity);

    // update kinematic model
    adKinematicsModel.update(basePose, qJoints);

    // Get foot position and Jacobian
    using ad_footJacobian_t = Eigen::Matrix<ad_scalar_t, 6, 12>;
    Vector3Ad com_base2StanceFeet_;
    ad_footJacobian_t b_feetJacobians_;
    adKinematicsModel.footPositionBaseFrame(legNumber_, com_base2StanceFeet_);  // base to stance feet displacement in the CoM frame
    adKinematicsModel.footJacobainBaseFrame(legNumber_, b_feetJacobians_);      // foot Jacobian's in the Base frame

    // Compute foot velocity
    Vector3Ad com_footVelocity =
        b_feetJacobians_.bottomRows(3) * dqJoints + com_baseLinearVelocity + com_baseAngularVelocity.cross(com_base2StanceFeet_);
    o_footVelocity = o_R_b_ * com_footVelocity;
  }

  void setAdInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel) {
    // Function to differentiate
    ad_funtion_t adfunc_ = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      this->adFootVelocity(adComModel, adKinematicsModel, x, y);
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
  }

  int legNumber_;
  EndEffectorVelocityConstraintSettings settings_;
  std::string libName_;
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> cppAdCodeGenClass_;
};
}  // namespace switched_model

#endif  // OCS2_CTRL_ENDEFFECTORVELOCITYCONTRAINT_H
