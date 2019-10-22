
#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

struct EndEffectorVelocityConstraintSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constraints are  A * v_world + b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

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

  using ad_interface_t = ocs2::CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using dynamic_vector_t = typename ad_interface_t::dynamic_vector_t;

  using constraint_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, -1, 1 + STATE_DIM + INPUT_DIM>;
  using timeStateInput_matrix_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM + INPUT_DIM, 1 + STATE_DIM + INPUT_DIM>;

 public:
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  explicit EndEffectorVelocityConstraint(int legNumber, EndEffectorVelocityConstraintSettings settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels)
      : BASE(ocs2::ConstraintOrder::Linear),
        legNumber_(legNumber),
        settings_(std::move(settings)),
        libName_("EEVelocityConstraint_" + std::to_string(legNumber_)),
        libFolder_("/tmp/ocs2") {
    setAdInterface(adComModel, adKinematicsModel);
    if (generateModels) {
      adInterface_->createModels(ad_interface_t::ApproximationOrder::First, true);
    } else {
      adInterface_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, true);
    }
  }

  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs)
      : BASE(rhs),
        legNumber_(rhs.legNumber_),
        settings_(rhs.settings_),
        libName_(rhs.libName_),
        libFolder_(rhs.libFolder_),
        adInterface_(new ad_interface_t(*rhs.adInterface_)) { }

  EndEffectorVelocityConstraint* clone() const override { return new EndEffectorVelocityConstraint(*this); }

  void configure(const EndEffectorVelocityConstraintSettings& settings) { settings_ = settings; };

  size_t getNumConstraints(scalar_t time) const override { return settings_.A.rows(); };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
    tapedInput << time, state, input;

    // Compute constraints
    dynamic_vector_t eeVelocityWorld = adInterface_->getFunctionValue(tapedInput);

    // Change to std::vector
    scalar_array_t constraintValue;
    for (int i = 0; i < settings_.A.rows(); i++) {
      constraintValue.emplace_back(settings_.A.row(i) * eeVelocityWorld + settings_.b[i]);
    }
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
    tapedInput << time, state, input;

    // Compute end effector velocity and derivatives
    constraint_timeStateInput_matrix_t stateInputJacobian = adInterface_->getJacobian(tapedInput);

    // Collect constraint terms
    auto dhdx = stateInputJacobian.template middleCols<STATE_DIM>(1);
    auto dhdu = stateInputJacobian.template rightCols<INPUT_DIM>();

    // Convert to output format
    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues = getValue(time, state, input);
    for (int i = 0; i < settings_.A.rows(); i++) {
      linearApproximation.derivativeState.emplace_back(settings_.A.row(i) * dhdx);
      linearApproximation.derivativeInput.emplace_back(settings_.A.row(i) * dhdu);
    }
    return linearApproximation;
  }

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
    tapedInput << time, state, input;

    // Convert to output format
    QuadraticApproximation_t quadraticApproximation;
    auto linearApproximation = getLinearApproximation(time, state, input);
    quadraticApproximation.constraintValues = std::move(linearApproximation.constraintValues);
    quadraticApproximation.derivativeState = std::move(linearApproximation.derivativeState);
    quadraticApproximation.derivativeInput = std::move(linearApproximation.derivativeInput);
    for (int i = 0; i < settings_.A.rows(); i++) {
      timeStateInput_matrix_t weightedHessian = adInterface_->getHessian(settings_.A.row(i), tapedInput);

      quadraticApproximation.secondDerivativesState.emplace_back(weightedHessian.block(1, 1, STATE_DIM, STATE_DIM));
      quadraticApproximation.secondDerivativesInput.emplace_back(weightedHessian.block(1 + STATE_DIM, 1 + STATE_DIM, INPUT_DIM, INPUT_DIM));
      quadraticApproximation.derivativesInputState.emplace_back(weightedHessian.block(1 + STATE_DIM, 1, INPUT_DIM, STATE_DIM));
    }
    return quadraticApproximation;
  }

 private:
  void adFootVelocity(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                      ad_dynamic_vector_t& o_footVelocity) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    ad_dynamic_vector_t x = tapedInput.segment(1, STATE_DIM);
    ad_dynamic_vector_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

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
    Vector3Ad com_base2CoM_ = adComModel.comPositionBaseFrame();
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
    auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      this->adFootVelocity(adComModel, adKinematicsModel, x, y);
    };

    adInterface_.reset(new ad_interface_t(adfunc, range_dim_, domain_dim_, libName_, libFolder_));
  }

  int legNumber_;
  EndEffectorVelocityConstraintSettings settings_;
  std::string libName_;
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> adInterface_;
};
}  // namespace switched_model
