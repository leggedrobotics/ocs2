

#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

struct EndEffectorPositionConstraintSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Desired end effector position specified as h(x,u) = A*p(x,u) + b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

class EndEffectorPositionConstraint final : public ocs2::ConstraintTerm<24, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t STATE_DIM = 24;
  static constexpr size_t INPUT_DIM = 24;

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
  using ad_kinematic_model_t = KinematicsModelBase<12, ad_scalar_t>;

  explicit EndEffectorPositionConstraint(int legNumber, EndEffectorPositionConstraintSettings settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels)
      : BASE(ocs2::ConstraintOrder::Quadratic),
        legNumber_(legNumber),
        settings_(std::move(settings)),
        libName_("EEPositionConstraint_" + std::to_string(legNumber_)),
        libFolder_("/tmp/ocs2") {
    setAdInterface(adComModel, adKinematicsModel);
    setAdInterface(adComModel, adKinematicsModel);
    if (generateModels) {
      adInterface_->createModels(ad_interface_t::ApproximationOrder::Second, true);
    } else {
      adInterface_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, true);
    }
  }

  EndEffectorPositionConstraint(const EndEffectorPositionConstraint& rhs)
      : BASE(rhs),
        legNumber_(rhs.legNumber_),
        settings_(rhs.settings_),
        libName_(rhs.libName_),
        libFolder_(rhs.libFolder_),
        adInterface_(new ad_interface_t(*rhs.adInterface_)) { }

  EndEffectorPositionConstraint* clone() const override {
    return new EndEffectorPositionConstraint(*this);
  }

  void configure(const EndEffectorPositionConstraintSettings& settings) { settings_ = settings; };

  size_t getNumConstraints(scalar_t time) const override { return settings_.A.rows(); };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
    tapedInput << time, state, input;

    // Compute constraints
    dynamic_vector_t eePositionWorld = adInterface_->getFunctionValue(tapedInput);

    // Change to std::vector
    scalar_array_t constraintValue;
    for (int i = 0; i < settings_.A.rows(); i++) {
      constraintValue.emplace_back(settings_.A.row(i) * eePositionWorld + settings_.b[i]);
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
  void adFootPosition(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                      ad_dynamic_vector_t& o_footPosition) {
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
    Vector3Ad o_comPosition = x.segment(3, 3);  // in origin frame
    ad_joint_coordinate_t qJoints = x.segment(12, 12);

    // base coordinates [EulerAngles, base position in world]
    Vector3Ad com_base2CoM_ = adComModel.comPositionBaseFrame();
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
    auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      this->adFootPosition(adComModel, adKinematicsModel, x, y);
    };

    adInterface_.reset(new ad_interface_t(adfunc, range_dim_, domain_dim_, libName_, libFolder_));
  };

  int legNumber_;
  EndEffectorPositionConstraintSettings settings_;
  std::string libName_;
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> adInterface_;
};
}  // namespace switched_model
