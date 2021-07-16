/******************************************************************************
 * File:             EndEffectorConstraint.cpp
 *
 * Author:           Oliver Harley
 * Created:          03/16/20
 * Description:      Endeffector constraints implementation.
 *****************************************************************************/

#include "ocs2_switched_model_interface/constraint/EndEffectorConstraint.h"

namespace switched_model {

EndEffectorConstraintSettings::EndEffectorConstraintSettings(size_t rows, size_t cols)
    : b(Eigen::VectorXd(rows)), A(Eigen::MatrixXd(rows, cols)){};

void EndEffectorConstraintSettings::resize(size_t rows, size_t cols) {
  b.resize(rows);
  A.resize(rows, cols);
};

EndEffectorConstraint::EndEffectorConstraint(const std::string& eeConstraintName, int legNumber, EndEffectorConstraintSettings settings,
                                             ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, adfunc_t adfunc,
                                             bool generateModels)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), settings_(std::move(settings)), legStartIdx_(3 * legNumber) {
  const std::string modelName{eeConstraintName + std::to_string(legNumber)};
  const std::string modelFolder{"/tmp/ocs2"};
  const size_t domainDim = 1 + STATE_DIM + INPUT_DIM;

  auto diffFunc = [&](const ocs2::ad_vector_t& x, ocs2::ad_vector_t& y) { adfunc(adComModel, adKinematicsModel, legNumber, x, y); };
  adInterface_.reset(new ocs2::CppAdInterface(diffFunc, domainDim, modelName, modelFolder));

  const auto orderCppAd = ocs2::CppAdInterface::ApproximationOrder::First;
  if (generateModels) {
    adInterface_->createModels(orderCppAd, true);
  } else {
    adInterface_->loadModelsIfAvailable(orderCppAd, true);
  }
}

EndEffectorConstraint::EndEffectorConstraint(const EndEffectorConstraint& rhs)
    : ocs2::StateInputConstraint(rhs),
      settings_(rhs.settings_),
      legStartIdx_(rhs.legStartIdx_),
      adInterface_(new ocs2::CppAdInterface(*rhs.adInterface_)){};

EndEffectorConstraint* EndEffectorConstraint::clone() const {
  return new EndEffectorConstraint(*this);
};

void EndEffectorConstraint::configure(const EndEffectorConstraintSettings& settings) {
  settings_ = settings;
};

size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
  return settings_.A.rows();
};

vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  // Assemble input
  vector_t tapedInput(1 + state.rows() + input.rows());
  tapedInput << time, state, input;

  // Compute taped function
  const vector_t funcVal = adInterface_->getFunctionValue(tapedInput);

  vector_t constraintValues = settings_.b;
  constraintValues.noalias() += settings_.A * funcVal;
  return constraintValues;
}

VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const vector_t& input) const {
  const auto numConstraints = settings_.A.rows();

  // Assemble input
  vector_t tapedInput(1 + state.rows() + input.rows());
  tapedInput << time, state, input;

  // Compute end effector velocity and derivatives
  const auto funcVal = adInterface_->getFunctionValue(tapedInput);
  const auto stateInputJacobian = adInterface_->getJacobian(tapedInput);

  // Convert to output format
  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = settings_.b;
  linearApproximation.f.noalias() += settings_.A * funcVal;

  // State Derivative - base part
  linearApproximation.dfdx.resize(numConstraints, STATE_DIM);
  const auto baseStateSize = 2 * BASE_COORDINATE_SIZE;
  linearApproximation.dfdx.leftCols<baseStateSize>().noalias() = settings_.A * stateInputJacobian.middleCols<baseStateSize>(1);

  // State Derivative - joints positions part
  linearApproximation.dfdx.middleCols<JOINT_COORDINATE_SIZE>(baseStateSize).setZero();
  const auto jointPositionInStateIdx = baseStateSize + legStartIdx_;
  linearApproximation.dfdx.middleCols<3>(jointPositionInStateIdx).noalias() =
      settings_.A * stateInputJacobian.middleCols<3>(1 + jointPositionInStateIdx);

  // Input Derivative
  linearApproximation.dfdu.setZero(numConstraints, INPUT_DIM);
  const auto jointVelInInputIdx = 12 + legStartIdx_;
  linearApproximation.dfdu.middleCols<3>(jointVelInInputIdx).noalias() =
      settings_.A * stateInputJacobian.middleCols<3>(1 + STATE_DIM + jointVelInInputIdx);  // Joint velocities of this leg

  return linearApproximation;
}

}  // namespace switched_model
