//
// Created by rgrandia on 26.06.20.
//

#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"

namespace switched_model {

FootPlacementCost::FootPlacementCost(FootPlacementCostParameters settings, const ad_com_model_t& adComModel,
                                     const ad_kinematic_model_t& adKinematicsModel, bool generateModels)
    : settings_(settings), constraintValuesUpdated_(false), feetJacobiansUpdated_(false) {
  std::string libName = "FootPlacementCost";
  std::string libFolder = "/tmp/ocs2";
  auto diffFunc = [&](const ad_vector_t& x, ad_vector_t& y) { adfunc(adComModel, adKinematicsModel, x, y); };
  adInterface_.reset(new ad_interface_t(diffFunc, STATE_DIM, libName, libFolder));
  initAdModels(generateModels);
}

FootPlacementCost::FootPlacementCost(const FootPlacementCost& rhs)
    : settings_(rhs.settings_),
      constraintValuesUpdated_(false),
      feetJacobiansUpdated_(false),
      adInterface_(new ad_interface_t(*rhs.adInterface_)) {}

FootPlacementCost* FootPlacementCost::clone() const {
  return new FootPlacementCost(*this);
}

void FootPlacementCost::setStateAndConstraint(const vector_t& x, const feet_array_t<const FootTangentialConstraintMatrix*>& constraints) {
  x_ = x;
  constraints_ = constraints;
  constraintValuesUpdated_ = false;
  feetJacobiansUpdated_ = false;
}

scalar_t FootPlacementCost::getCostValue() {
  updateConstraintValues();

  scalar_t cost(0.0);
  for (const auto& h : constraintValues_) {  // Loop through the legs
    for (int j = 0; j < h.size(); ++j) {     // Loop through all faces of the terrain constraint
      cost += getPenaltyFunctionValue(h(j), settings_);
    }
  }
  return cost;
}

vector_t FootPlacementCost::getCostDerivativeState() {
  updateConstraintValues();
  updateJacobians();

  vector_t costDerivative = vector_t::Zero(STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = constraints_[leg];
    if (constraintMatrixPtr != nullptr) {
      const auto& h = constraintValues_[leg];
      const auto penaltyDerivatives = h.unaryExpr([&](scalar_t hi) { return getPenaltyFunctionDerivative(hi, settings_); });
      vector3_t tmp = constraintMatrixPtr->A.transpose() * penaltyDerivatives;
      costDerivative.noalias() += feetJacobiansInOrigin_[leg].transpose() * tmp;
    }
  }
  return costDerivative;
}

matrix_t FootPlacementCost::getCostSecondDerivativeState() {
  updateConstraintValues();
  updateJacobians();

  matrix_t costSecondDerivative = matrix_t::Zero(STATE_DIM, STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = constraints_[leg];
    if (constraintMatrixPtr != nullptr) {
      const auto& h = constraintValues_[leg];
      const auto penaltySecondDerivatives = h.unaryExpr([&](scalar_t hi) { return getPenaltyFunctionSecondDerivative(hi, settings_); });
      matrix3_t tmp = constraintMatrixPtr->A.transpose() * penaltySecondDerivatives.asDiagonal() * constraintMatrixPtr->A;
      costSecondDerivative.noalias() += feetJacobiansInOrigin_[leg].transpose() * tmp * feetJacobiansInOrigin_[leg];
    }
  }

  return costSecondDerivative;
}

void FootPlacementCost::initAdModels(bool generateModels, bool verbose) {
  auto order = ad_interface_t::ApproximationOrder::First;
  if (generateModels) {
    adInterface_->createModels(order, verbose);
  } else {
    adInterface_->loadModelsIfAvailable(order, verbose);
  }
}

void FootPlacementCost::adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_vector_t& state,
                               ad_vector_t& o_feetPositions) {
  // Copy to fixed size
  comkino_state_ad_t x = state;

  // Extract elements from state
  const base_coordinate_ad_t comPose = getComPose(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);

  // Get base state from com state
  const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);
  const auto o_feetPositionsAsArray = adKinematicsModel.feetPositionsInOriginFrame(basePose, qJoints);
  o_feetPositions = fromArray(o_feetPositionsAsArray);
}

void FootPlacementCost::updateJacobians() {
  if (!feetJacobiansUpdated_) {
    const auto stackedJacobians = adInterface_->getJacobian(x_);
    feetJacobiansInOrigin_[0] = stackedJacobians.block<3, STATE_DIM>(0, 0);
    feetJacobiansInOrigin_[1] = stackedJacobians.block<3, STATE_DIM>(3, 0);
    feetJacobiansInOrigin_[2] = stackedJacobians.block<3, STATE_DIM>(6, 0);
    feetJacobiansInOrigin_[3] = stackedJacobians.block<3, STATE_DIM>(9, 0);
    feetJacobiansUpdated_ = true;
  }
}

void FootPlacementCost::updateConstraintValues() {
  if (!constraintValuesUpdated_) {
    const auto feetPositionsInOrigin_ = adInterface_->getFunctionValue(x_);

    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      const auto* constraintMatrixPtr = constraints_[leg];
      if (constraintMatrixPtr != nullptr) {
        constraintValues_[leg] = constraintMatrixPtr->b;
        constraintValues_[leg].noalias() += constraintMatrixPtr->A * feetPositionsInOrigin_.segment<3>(3 * leg);
      } else {
        constraintValues_[leg].resize(0);
      }
    }
    constraintValuesUpdated_ = true;
  }
}

scalar_t FootPlacementCost::getPenaltyFunctionValue(scalar_t h, const FootPlacementCostParameters& config) {
  if (h > config.delta) {
    return -config.mu * log(h);
  } else {
    auto tmp = (h - 2.0 * config.delta) / config.delta;
    return config.mu * (-log(config.delta) + scalar_t(0.5) * tmp * tmp - scalar_t(0.5));
  }
}

scalar_t FootPlacementCost::getPenaltyFunctionDerivative(scalar_t h, const FootPlacementCostParameters& config) {
  if (h > config.delta) {
    return -config.mu / h;
  } else {
    return config.mu * ((h - 2.0 * config.delta) / (config.delta * config.delta));
  }
}

scalar_t FootPlacementCost::getPenaltyFunctionSecondDerivative(scalar_t h, const FootPlacementCostParameters& config) {
  if (h > config.delta) {
    return config.mu / (h * h);
  } else {
    return config.mu / (config.delta * config.delta);
  }
}

}  // namespace switched_model