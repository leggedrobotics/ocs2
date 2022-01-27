//
// Created by rgrandia on 06.08.20.
//

#include <ocs2_switched_model_interface/constraint/FrictionConeConstraint.h>

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

FrictionConeConstraint::FrictionConeConstraint(Config config, int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Quadratic),
      config_(std::move(config)),
      legNumber_(legNumber),
      modeScheduleManager_(&modeScheduleManager) {}

matrix3_t FrictionConeConstraint::getRotationWorldToTerrain(const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  return orientationWorldToTerrainFromSurfaceNormalInWorld(switchedModelPreComp.getSurfaceNormalInOriginFrame(legNumber_));
}

bool FrictionConeConstraint::isActive(scalar_t time) const {
  return modeScheduleManager_->getContactFlags(time)[legNumber_];
}

vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const ocs2::PreComputation& preComp) const {
  const matrix3_t t_R_w = getRotationWorldToTerrain(preComp);

  const comkino_state_t comkinoState = state;
  const vector3_t eulerXYZ = getOrientation(getBasePose(comkinoState));
  const vector3_t forcesInBodyFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = computeLocalForces(t_R_w, eulerXYZ, forcesInBodyFrame);

  return coneConstraint(localForce);
}

VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const ocs2::PreComputation& preComp) const {
  const matrix3_t t_R_w = getRotationWorldToTerrain(preComp);

  const comkino_state_t comkinoState = state;
  const vector3_t eulerXYZ = getOrientation(getBasePose(comkinoState));
  const vector3_t forcesInBodyFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = computeLocalForces(t_R_w, eulerXYZ, forcesInBodyFrame);
  const auto localForceDerivatives = computeLocalForceDerivatives(t_R_w, eulerXYZ, forcesInBodyFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = coneConstraint(localForce);
  linearApproximation.dfdx = frictionConeStateDerivative(coneDerivatives);
  linearApproximation.dfdu = frictionConeInputDerivative(coneDerivatives);
  return linearApproximation;
}

VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const ocs2::PreComputation& preComp) const {
  const matrix3_t t_R_w = getRotationWorldToTerrain(preComp);

  const comkino_state_t comkinoState = state;
  const vector3_t eulerXYZ = getOrientation(getBasePose(comkinoState));
  const vector3_t forcesInBodyFrame = input.segment(3 * legNumber_, 3);

  const auto localForce = computeLocalForces(t_R_w, eulerXYZ, forcesInBodyFrame);
  const auto localForceDerivatives = computeLocalForceDerivatives(t_R_w, eulerXYZ, forcesInBodyFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = coneConstraint(localForce);
  quadraticApproximation.dfdx = frictionConeStateDerivative(coneDerivatives);
  quadraticApproximation.dfdu = frictionConeInputDerivative(coneDerivatives);
  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(coneDerivatives));
  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(coneDerivatives));
  quadraticApproximation.dfdux.emplace_back(frictionConeDerivativesInputState(coneDerivatives));
  return quadraticApproximation;
}

FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const matrix3_t& t_R_w, const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const {
  LocalForceDerivatives localForceDerivatives{};
  localForceDerivatives.dF_deuler = t_R_w * rotationBaseToOriginJacobian(eulerXYZ, forcesInBodyFrame);
  localForceDerivatives.dF_du = t_R_w * rotationMatrixBaseToOrigin(eulerXYZ);
  return localForceDerivatives;
}

vector3_t FrictionConeConstraint::computeLocalForces(const matrix3_t& t_R_w, const vector3_t& eulerXYZ,
                                                     const vector3_t& forcesInBodyFrame) const {
  const vector3_t forcesInWorld = rotateVectorBaseToOrigin(forcesInBodyFrame, eulerXYZ);
  return t_R_w * forcesInWorld;
}

FrictionConeConstraint::ConeLocalDerivatives FrictionConeConstraint::computeConeLocalDerivatives(const vector3_t& localForces) const {
  const auto F_x_square = localForces.x() * localForces.x();
  const auto F_y_square = localForces.y() * localForces.y();
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

  ConeLocalDerivatives coneDerivatives{};
  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

  return coneDerivatives;
}

vector_t FrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);

  ocs2::vector_t h(1);
  h[0] = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
  return h;
}

FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  // First order derivatives
  coneDerivatives.dCone_deuler.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_deuler;
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

  // Second order derivatives
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;
  coneDerivatives.d2Cone_deuler2.noalias() =
      localForceDerivatives.dF_deuler.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_deuler;
  coneDerivatives.d2Cone_dudeuler.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_deuler;

  // Hessian shift
  coneDerivatives.d2Cone_du2.diagonal().array() -= config_.hessianDiagonalShift;
  coneDerivatives.d2Cone_deuler2.diagonal().array() -= config_.hessianDiagonalShift;

  return coneDerivatives;
}

matrix_t FrictionConeConstraint::frictionConeStateDerivative(const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdx = matrix_t::Zero(1, STATE_DIM);
  dhdx.block<1, 3>(0, 0) = coneDerivatives.dCone_deuler;
  return dhdx;
}

matrix_t FrictionConeConstraint::frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, INPUT_DIM);
  dhdu.block<1, 3>(0, 3 * legNumber_) = coneDerivatives.dCone_du;
  return dhdu;
}

matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(INPUT_DIM, INPUT_DIM);
  ddhdudu.block<3, 3>(3 * legNumber_, 3 * legNumber_) = coneDerivatives.d2Cone_du2;
  return ddhdudu;
}

matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdxdx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  ddhdxdx.block<3, 3>(0, 0) = coneDerivatives.d2Cone_deuler2;
  return ddhdxdx;
}

matrix_t FrictionConeConstraint::frictionConeDerivativesInputState(const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudx = matrix_t::Zero(INPUT_DIM, STATE_DIM);
  ddhdudx.block<3, 3>(3 * legNumber_, 0) = coneDerivatives.d2Cone_dudeuler;
  return ddhdudx;
}

}  // namespace switched_model
