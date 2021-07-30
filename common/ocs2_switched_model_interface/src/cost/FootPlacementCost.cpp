//
// Created by rgrandia on 26.06.20.
//

#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"

namespace switched_model {

FootPlacementCost::FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings, ocs2::RelaxedBarrierPenalty::Config sdfSettings,
                                     const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                     const com_model_t& comModel, const ad_com_model_t& adComModel, bool generateModels)
    : polygonPenalty_(new ocs2::RelaxedBarrierPenalty(settings)), sdfPenalty_(new ocs2::RelaxedBarrierPenalty(sdfSettings)) {
  std::string libName = "FootPlacementCost";
  std::string libFolder = "/tmp/ocs2";
  auto diffFunc = [&](const ad_vector_t& x, ad_vector_t& y) { adfunc(adComModel, adKinematicModel, x, y); };
  adInterface_.reset(new ad_interface_t(diffFunc, STATE_DIM, libName, libFolder));

  // Get collision body info from kinematics
  const auto collisions = kinematicModel.collisionSpheresInBaseFrame(joint_coordinate_t::Zero());

  // Initialize data for all targets
  const int numberOfTargets = NUM_CONTACT_POINTS + collisions.size();
  targetPositions_.resize(numberOfTargets);
  targetJacobians_.resize(numberOfTargets);
  constraintsPerTarget_.resize(numberOfTargets);
  targetRadii_ = std::vector<scalar_t>(NUM_CONTACT_POINTS, 0.0);  // fill zero radius for the feet.
  for (const auto& sphere : collisions) {
    targetRadii_.push_back(sphere.radius);
  }

  // Generate the model
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
      targetPositions_(rhs.targetPositions_),
      targetJacobians_(rhs.targetJacobians_),
      constraintsPerTarget_(rhs.constraintsPerTarget_),
      targetRadii_(rhs.targetRadii_),
      adInterface_(new ad_interface_t(*rhs.adInterface_)) {}

FootPlacementCost* FootPlacementCost::clone() const {
  return new FootPlacementCost(*this);
}

void FootPlacementCost::setConstraints(const feet_array_t<const FootTangentialConstraintMatrix*>& constraints,
                                       const feet_array_t<SignedDistanceConstraint>& sdfConstraints, const SignedDistanceField* sdfPtr) {
  constraints_ = constraints;
  sdfConstraints_ = sdfConstraints;
  sdfPtr_ = sdfPtr;
}

scalar_t FootPlacementCost::getValue(const vector_t& x) {
  updatePositions(x);
  updateConstraintValues(x);

  scalar_t cost(0.0);
  for (size_t target = 0; target < targetPositions_.size(); ++target) {
    if (!constraintsPerTarget_[target].empty()) {
      cost += switched_model::getValue(constraintsPerTarget_[target], targetPositions_[target]);
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation FootPlacementCost::getQuadraticApproximation(const vector_t& x) {
  updatePositions(x);
  updateConstraintValues(x);
  updateJacobians(x);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);

  for (size_t target = 0; target < targetPositions_.size(); ++target) {
    if (!constraintsPerTarget_[target].empty()) {
      const auto targetcost =
          switched_model::getQuadraticApproximation(constraintsPerTarget_[target], targetPositions_[target], targetJacobians_[target]);
      cost.f += targetcost.f;
      cost.dfdx += targetcost.dfdx;
      cost.dfdxx += targetcost.dfdxx;
    }
  }

  return cost;
}

void FootPlacementCost::adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_vector_t& state,
                               ad_vector_t& targetsInOriginFrame) {
  // Copy to fixed size
  comkino_state_ad_t x = state;

  // Extract elements from state
  const base_coordinate_ad_t basePose = getComPose(x);
  const joint_coordinate_ad_t qJoints = getJointPositions(x);

  const auto o_feetPositionsAsArray = adKinematicsModel.feetPositionsInOriginFrame(basePose, qJoints);
  const auto o_collisions = adKinematicsModel.collisionSpheresInOriginFrame(basePose, qJoints);

  const int numberOfTargets = NUM_CONTACT_POINTS + o_collisions.size();
  targetsInOriginFrame.resize(3 * numberOfTargets);

  targetsInOriginFrame.head<3 * NUM_CONTACT_POINTS>() = fromArray(o_feetPositionsAsArray);

  int i = 3 * NUM_CONTACT_POINTS;
  for (const auto& sphere : o_collisions) {
    targetsInOriginFrame.segment<3>(i) = sphere.position;
    i += 3;
  }
}

void FootPlacementCost::updatePositions(const vector_t& x) {
  const auto stackedPositions = adInterface_->getFunctionValue(x);
  for (size_t target = 0; target < targetPositions_.size(); ++target) {
    targetPositions_[target] = stackedPositions.segment<3>(3 * target);
  }
}

void FootPlacementCost::updateJacobians(const vector_t& x) {
  const auto stackedJacobians = adInterface_->getJacobian(x);
  for (size_t target = 0; target < targetJacobians_.size(); ++target) {
    targetJacobians_[target] = stackedJacobians.block<3, STATE_DIM>(3 * target, 0);
  }
}

void FootPlacementCost::updateConstraintValues(const vector_t& x) {
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    auto& constraints = constraintsPerTarget_[leg];
    constraints.clear();

    const auto* constraintMatrixPtr = constraints_[leg];
    if (constraintMatrixPtr != nullptr) {
      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = polygonPenalty_.get();
      linearStateInequalitySoftConstraint.A = constraintMatrixPtr->A;
      linearStateInequalitySoftConstraint.h = constraintMatrixPtr->b;
      linearStateInequalitySoftConstraint.h.noalias() += constraintMatrixPtr->A * targetPositions_[leg];
      constraints.push_back(std::move(linearStateInequalitySoftConstraint));
    }

    const auto* sdfPtr = sdfConstraints_[leg].signedDistanceField;
    if (sdfPtr != nullptr) {
      const auto sdfFirstOrder = sdfPtr->valueAndDerivative(targetPositions_[leg]);
      const auto h_sdf = sdfFirstOrder.first - sdfConstraints_[leg].minimumDistance;

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = sdfPenalty_.get();
      linearStateInequalitySoftConstraint.A = sdfFirstOrder.second.transpose();
      linearStateInequalitySoftConstraint.h = (vector_t(1) << h_sdf).finished();
      constraints.push_back(std::move(linearStateInequalitySoftConstraint));
    }
  }

  for (int target = NUM_CONTACT_POINTS; target < constraintsPerTarget_.size(); ++target) {
    auto& constraints = constraintsPerTarget_[target];
    constraints.clear();

    if (sdfPtr_ != nullptr) {
      const auto sdfFirstOrder = sdfPtr_->valueAndDerivative(targetPositions_[target]);
      const auto h_sdf = sdfFirstOrder.first - targetRadii_[target];

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = sdfPenalty_.get();
      linearStateInequalitySoftConstraint.A = sdfFirstOrder.second.transpose();
      linearStateInequalitySoftConstraint.h = (vector_t(1) << h_sdf).finished();
      constraints.push_back(std::move(linearStateInequalitySoftConstraint));
    }
  }
}

}  // namespace switched_model
