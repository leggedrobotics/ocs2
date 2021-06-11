//
// Created by rgrandia on 06.08.20.
//

#include <ocs2_switched_model_interface/constraint/FrictionConeConstraint.h>

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

FrictionConeConstraint::FrictionConeConstraint(Config config, int legNumber)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Quadratic),
      config_(std::move(config)),
      legNumber_(legNumber),
      t_R_w(matrix3_t::Identity()) {
  assert(config_.frictionCoefficient > 0.0);
  assert(config_.regularization > 0.0);
  assert(config_.hessianDiagonalShift >= 0.0);
}

void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w = orientationWorldToTerrainFromSurfaceNormalInWorld(surfaceNormalInWorld);
}

vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  const vector3_t eulerXYZ = getOrientation(getComPose(state));
  const vector3_t forcesInBodyFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);

  return (ocs2::vector_t(1) << coneConstraint(localForce)).finished();
}

VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input) const {
  const vector3_t eulerXYZ = getOrientation(getComPose(state));
  const vector3_t forcesInBodyFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);
  const auto localForceDerivatives = computeLocalForceDerivatives(eulerXYZ, forcesInBodyFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionLinearApproximation linearApproximation(1, STATE_DIM, INPUT_DIM);
  linearApproximation.f(0) = coneConstraint(localForce);
  linearApproximation.dfdx = frictionConeStateDerivative(coneDerivatives).transpose();
  linearApproximation.dfdu = frictionConeInputDerivative(coneDerivatives).transpose();
  return linearApproximation;
}

VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input) const {
  const vector3_t eulerXYZ = getOrientation(getComPose(state));
  const vector3_t forcesInBodyFrame = input.segment(3 * legNumber_, 3);

  const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);
  const auto localForceDerivatives = computeLocalForceDerivatives(eulerXYZ, forcesInBodyFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionQuadraticApproximation quadraticApproximation(1, STATE_DIM, INPUT_DIM);
  quadraticApproximation.f(0) = coneConstraint(localForce);
  quadraticApproximation.dfdx = frictionConeStateDerivative(coneDerivatives).transpose();
  quadraticApproximation.dfdu = frictionConeInputDerivative(coneDerivatives).transpose();
  quadraticApproximation.dfdxx[0] = frictionConeSecondDerivativeState(coneDerivatives);
  quadraticApproximation.dfduu[0] = frictionConeSecondDerivativeInput(coneDerivatives);
  quadraticApproximation.dfdux[0] = frictionConeDerivativesInputState(coneDerivatives);
  return quadraticApproximation;
}

FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const {
  LocalForceDerivatives localForceDerivatives{};
  localForceDerivatives.dF_deuler = t_R_w * rotationBaseToOriginJacobian(eulerXYZ, forcesInBodyFrame);
  localForceDerivatives.dF_du = t_R_w * rotationMatrixBaseToOrigin(eulerXYZ);
  return localForceDerivatives;
}

vector3_t FrictionConeConstraint::computeLocalForces(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const {
  matrix3_t t_R_b = t_R_w * rotationMatrixBaseToOrigin(eulerXYZ);
  return t_R_b * forcesInBodyFrame;
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

scalar_t FrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);

  return config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
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

  return coneDerivatives;
}

state_vector_t FrictionConeConstraint::frictionConeStateDerivative(const ConeDerivatives& coneDerivatives) const {
  state_vector_t dhdx = state_vector_t::Zero();
  dhdx.segment<3>(0) = coneDerivatives.dCone_deuler;
  return dhdx;
}

input_vector_t FrictionConeConstraint::frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const {
  input_vector_t dhdu = input_vector_t::Zero();
  dhdu.segment<3>(3 * legNumber_) = coneDerivatives.dCone_du;
  return dhdu;
}

input_matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const {
  input_matrix_t ddhdudu = input_matrix_t::Zero();
  ddhdudu.block<3, 3>(3 * legNumber_, 3 * legNumber_) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

state_matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const {
  state_matrix_t ddhdxdx = state_matrix_t::Zero();
  ddhdxdx.block<3, 3>(0, 0) = coneDerivatives.d2Cone_deuler2;
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

input_state_matrix_t FrictionConeConstraint::frictionConeDerivativesInputState(const ConeDerivatives& coneDerivatives) const {
  input_state_matrix_t ddhdudx = input_state_matrix_t::Zero();
  ddhdudx.block<3, 3>(3 * legNumber_, 0) = coneDerivatives.d2Cone_dudeuler;
  return ddhdudx;
}

}  // namespace switched_model
