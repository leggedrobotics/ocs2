#include <ocs2_legged_robot_example/constraint/FrictionConeConstraint.h>

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::FrictionConeConstraint(Config config, int legNumber)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Quadratic),
      config_(std::move(config)),
      legNumber_(legNumber),
      t_R_w(matrix3_t::Identity()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w = switched_model::orientationWorldToTerrainFromSurfaceNormalInWorld(surfaceNormalInWorld);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  const vector3_t forcesInWorldFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = t_R_w * forcesInWorldFrame;

  return coneConstraint(localForce);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input) const {
  const vector3_t forcesInWorldFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = t_R_w * forcesInWorldFrame;
  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = coneConstraint(localForce);
  linearApproximation.dfdx = matrix_t::Zero(1, STATE_DIM_);
  linearApproximation.dfdu = frictionConeInputDerivative(coneDerivatives);
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input) const {
  const vector3_t forcesInWorldFrame = input.segment<3>(3 * legNumber_);

  const auto localForce = t_R_w * forcesInWorldFrame;
  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  ocs2::VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = coneConstraint(localForce);
  quadraticApproximation.dfdx = matrix_t::Zero(1, STATE_DIM_);
  quadraticApproximation.dfdu = frictionConeInputDerivative(coneDerivatives);
  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(coneDerivatives));
  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(coneDerivatives));
  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(INPUT_DIM_, STATE_DIM_));
  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const vector3_t& forcesInWorldFrame) const {
  LocalForceDerivatives localForceDerivatives{};
  localForceDerivatives.dF_du = t_R_w;
  return localForceDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
  return (ocs2::vector_t(1) << coneConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  // First order derivatives
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

  // Second order derivatives
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, INPUT_DIM_);
  dhdu.block<1, 3>(0, 3 * legNumber_) = coneDerivatives.dCone_du;
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(INPUT_DIM_, INPUT_DIM_);
  ddhdudu.block<3, 3>(3 * legNumber_, 3 * legNumber_) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdxdx = matrix_t::Zero(STATE_DIM_, STATE_DIM_);
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

}  // namespace legged_robot
}  // namespace ocs2
