/******************************************************************************
 * File:             EndEffectorConstraint.cpp
 *
 * Author:           Oliver Harley
 * Created:          03/16/20
 * Description:      Endeffector constraints implementation.
 *****************************************************************************/

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

namespace switched_model {

EndEffectorConstraintSettings::EndEffectorConstraintSettings(size_t rows, size_t cols)
    : b(Eigen::VectorXd(rows)), A(Eigen::MatrixXd(rows, cols)){};

void EndEffectorConstraintSettings::resize(size_t rows, size_t cols) {
  b.resize(rows);
  A.resize(rows, cols);
};

/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/

EndEffectorConstraint::EndEffectorConstraint(ocs2::ConstraintOrder constraintOrder, std::string eeConstraintName, int legNumber,
                                             EndEffectorConstraintSettings settings, ad_com_model_t& adComModel,
                                             ad_kinematic_model_t& adKinematicsModel, adfunc_t adfunc, bool generateModels, bool loadModels)
    : BASE(constraintOrder),
      legNumber_(legNumber),
      settings_(std::move(settings)),
      libName_(eeConstraintName + std::to_string(legNumber)),
      libFolder_("/tmp/ocs2") {
  auto diffFunc = [&](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) { adfunc(adComModel, adKinematicsModel, legNumber, x, y); };
  adInterface_.reset(new ad_interface_t(diffFunc, range_dim_, domain_dim_, libName_, libFolder_));
  assert(adInterface_);
  initAdModels(constraintOrder, generateModels, loadModels);
}

//! Note: Since the constructors are based on a copy we do not regenerate/generate the models
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorConstraint& rhs)
    : BASE(rhs),
      legNumber_(rhs.legNumber_),
      settings_(rhs.settings_),
      libName_(rhs.libName_),
      libFolder_("/tmp/ocs2"),
      adInterface_(new ad_interface_t(*rhs.adInterface_)){};

//! Note: Since the constructors are based on a copy we do not or regenerate/generate the models
EndEffectorConstraint* EndEffectorConstraint::clone() const {
  return new EndEffectorConstraint(*this);
};

void EndEffectorConstraint::configure(const EndEffectorConstraintSettings& settings) {
  settings_ = settings;
};

size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
  return settings_.A.rows();
};

EndEffectorConstraint::scalar_array_t EndEffectorConstraint::getValue(scalar_t time, const state_vector_t& state,
                                                                      const input_vector_t& input) const {
  // Assemble input
  dynamic_vector_t tapedInput(domain_dim_);
  tapedInput << time, state, input;

  // Compute constraints
  const dynamic_vector_t funcVal = adInterface_->getFunctionValue(tapedInput);

  // Change to std::vector
  scalar_array_t constraintValue;
  Eigen::VectorXd values = settings_.A * funcVal + settings_.b;
  for (int i = 0; i < settings_.A.rows(); i++) {
    constraintValue.emplace_back(values[i]);
  }
  return constraintValue;
};

EndEffectorConstraint::LinearApproximation_t EndEffectorConstraint::getLinearApproximation(scalar_t time, const state_vector_t& state,
                                                                                           const input_vector_t& input) const {
  // Assemble input
  dynamic_vector_t tapedInput(domain_dim_);
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

EndEffectorConstraint::QuadraticApproximation_t EndEffectorConstraint::getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                                                                 const input_vector_t& input) const {
  // Assemble input
  dynamic_vector_t tapedInput(domain_dim_);
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

void EndEffectorConstraint::initAdModels(ocs2::ConstraintOrder constraintOrder, bool generateModels, bool loadModels, bool verbose) {
  auto&& order = static_cast<ad_interface_t::ApproximationOrder>(constraintOrder);
  if (generateModels) {
    adInterface_->createModels(order, verbose);
  } else if (loadModels) {
    adInterface_->loadModelsIfAvailable(order, verbose);
  }
};

}  // namespace switched_model
