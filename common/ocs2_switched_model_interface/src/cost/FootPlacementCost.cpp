//
// Created by rgrandia on 26.06.20.
//

#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"

namespace switched_model {

FootPlacementCost::FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings, ocs2::RelaxedBarrierPenalty::Config sdfSettings,
                                     const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, bool generateModels)
    : polygonPenalty_(new ocs2::RelaxedBarrierPenalty(settings)), sdfPenalty_(new ocs2::RelaxedBarrierPenalty(sdfSettings)) {
  std::string libName = "FootPlacementCost";
  std::string libFolder = "/tmp/ocs2";
  auto diffFunc = [&](const ad_vector_t& x, ad_vector_t& y) { adfunc(adComModel, adKinematicsModel, x, y); };
  adInterface_.reset(new ad_interface_t(diffFunc, STATE_DIM, libName, libFolder));

  const bool verbose = true;
  const auto order = ad_interface_t::ApproximationOrder::First;
  if (generateModels) {
    adInterface_->createModels(order, verbose);
  } else {
    adInterface_->loadModelsIfAvailable(order, verbose);
  }
}

FootPlacementCost::FootPlacementCost(const FootPlacementCost& rhs)
    : polygonPenalty_(rhs.polygonPenalty_->clone()),
      sdfPenalty_(rhs.sdfPenalty_->clone()),
      adInterface_(new ad_interface_t(*rhs.adInterface_)) {}

FootPlacementCost* FootPlacementCost::clone() const {
  return new FootPlacementCost(*this);
}

void FootPlacementCost::setConstraints(const feet_array_t<const FootTangentialConstraintMatrix*>& constraints,
                                       const feet_array_t<SignedDistanceConstraint>& sdfConstraints) {
  constraints_ = constraints;
  sdfConstraints_ = sdfConstraints;
}

scalar_t FootPlacementCost::getValue(const vector_t& x) {
  updateConstraintValues(x);
  return getCostValue();
}

ScalarFunctionQuadraticApproximation FootPlacementCost::getQuadraticApproximation(const vector_t& x) {
  updateConstraintValues(x);
  updateJacobians(x);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = getCostValue();
  cost.dfdx = getCostDerivativeState();
  cost.dfdxx = getCostSecondDerivativeState();
  return cost;
}

scalar_t FootPlacementCost::getCostValue() const {
  scalar_t cost(0.0);
  for (const auto& h : constraintValues_) {  // Loop through the legs
    for (int j = 0; j < h.size(); ++j) {     // Loop through all faces of the terrain constraint
      cost += polygonPenalty_->getValue(h(j));
    }
  }

  // signed distance cost
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (sdfConstraints_[leg].signedDistanceField != nullptr) {
      const auto h_sdf = sdfValues_[leg].first - sdfConstraints_[leg].minimumDistance;
      cost += sdfPenalty_->getValue(h_sdf);
    }
  }

  return cost;
}

vector_t FootPlacementCost::getCostDerivativeState() const {
  vector_t costDerivative = vector_t::Zero(STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = constraints_[leg];
    const auto* sdfPtr = sdfConstraints_[leg].signedDistanceField;
    if (constraintMatrixPtr != nullptr || sdfPtr != nullptr) {
      vector3_t taskSpaceDerivative = vector3_t::Zero();

      // Tangential constraint contribution
      if (constraintMatrixPtr != nullptr) {
        const auto& h = constraintValues_[leg];
        const auto penaltyDerivatives = h.unaryExpr([&](scalar_t hi) { return polygonPenalty_->getDerivative(hi); });
        taskSpaceDerivative.noalias() += constraintMatrixPtr->A.transpose() * penaltyDerivatives;
      }

      // Sdf constraint contribution
      if (sdfPtr != nullptr) {
        const auto h_sdf = sdfValues_[leg].first - sdfConstraints_[leg].minimumDistance;
        const auto penaltyDerivative = sdfPenalty_->getDerivative(h_sdf);
        taskSpaceDerivative.noalias() += penaltyDerivative * sdfValues_[leg].second;
      }

      costDerivative.noalias() += feetJacobiansInOrigin_[leg].transpose() * taskSpaceDerivative;
    }
  }

  return costDerivative;
}

matrix_t FootPlacementCost::getCostSecondDerivativeState() const {
  matrix_t costSecondDerivative = matrix_t::Zero(STATE_DIM, STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = constraints_[leg];
    const auto* sdfPtr = sdfConstraints_[leg].signedDistanceField;
    if (constraintMatrixPtr != nullptr || sdfPtr != nullptr) {
      matrix3_t taskSpaceSecondDerivative = matrix3_t::Zero();

      // Tangential constraint contribution
      if (constraintMatrixPtr != nullptr) {
        const auto& h = constraintValues_[leg];
        const auto penaltySecondDerivatives = h.unaryExpr([&](scalar_t hi) { return polygonPenalty_->getSecondDerivative(hi); });
        taskSpaceSecondDerivative.noalias() +=
            constraintMatrixPtr->A.transpose() * penaltySecondDerivatives.asDiagonal() * constraintMatrixPtr->A;
      }

      // Sdf constraint contribution
      if (sdfPtr != nullptr) {
        const auto h_sdf = sdfValues_[leg].first - sdfConstraints_[leg].minimumDistance;
        const auto penaltySecondDerivative = sdfPenalty_->getSecondDerivative(h_sdf);
        taskSpaceSecondDerivative.noalias() += penaltySecondDerivative * sdfValues_[leg].second * sdfValues_[leg].second.transpose();
      }

      costSecondDerivative.noalias() += feetJacobiansInOrigin_[leg].transpose() * taskSpaceSecondDerivative * feetJacobiansInOrigin_[leg];
    }
  }

  return costSecondDerivative;
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

void FootPlacementCost::updateJacobians(const vector_t& x) {
  const auto stackedJacobians = adInterface_->getJacobian(x);
  feetJacobiansInOrigin_[0] = stackedJacobians.block<3, STATE_DIM>(0, 0);
  feetJacobiansInOrigin_[1] = stackedJacobians.block<3, STATE_DIM>(3, 0);
  feetJacobiansInOrigin_[2] = stackedJacobians.block<3, STATE_DIM>(6, 0);
  feetJacobiansInOrigin_[3] = stackedJacobians.block<3, STATE_DIM>(9, 0);
}

void FootPlacementCost::updateConstraintValues(const vector_t& x) {
  const auto feetPositionsInOrigin_ = adInterface_->getFunctionValue(x);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = constraints_[leg];
    if (constraintMatrixPtr != nullptr) {
      constraintValues_[leg] = constraintMatrixPtr->b;
      constraintValues_[leg].noalias() += constraintMatrixPtr->A * feetPositionsInOrigin_.segment<3>(3 * leg);
    } else {
      constraintValues_[leg].resize(0);
    }

    const auto* sdfPtr = sdfConstraints_[leg].signedDistanceField;
    if (sdfPtr != nullptr) {
      sdfValues_[leg] = sdfPtr->valueAndDerivative(feetPositionsInOrigin_.segment<3>(3 * leg));
    } else {
      sdfValues_[leg] = {scalar_t(0.0), vector3_t::Zero()};
    }
  }
}

}  // namespace switched_model
