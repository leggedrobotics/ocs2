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
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), settings_(std::move(settings)) {
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
    : ocs2::StateInputConstraint(rhs), settings_(rhs.settings_), adInterface_(new ocs2::CppAdInterface(*rhs.adInterface_)){};

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
  linearApproximation.dfdx.noalias() = settings_.A * stateInputJacobian.middleCols<STATE_DIM>(1);
  linearApproximation.dfdu.noalias() = settings_.A * stateInputJacobian.rightCols<INPUT_DIM>();

  return linearApproximation;
}

}  // namespace switched_model
