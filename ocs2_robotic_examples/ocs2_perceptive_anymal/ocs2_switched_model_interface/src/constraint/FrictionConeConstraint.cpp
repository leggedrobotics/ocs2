//
// Created by rgrandia on 06.08.20.
//

#include <ocs2_switched_model_interface/constraint/FrictionConeConstraint.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

namespace friction_cone {
scalar_t frictionConeConstraint(const Config& config, const vector3_t& forcesInTerrainFrame) {
  const auto& localForces = forcesInTerrainFrame;

  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config.regularization;
  const auto F_tangent_norm = std::sqrt(F_tangent_square);

  return config.frictionCoefficient * (localForces.z() + config.gripperForce) - F_tangent_norm;
}

ConeLocalDerivatives frictionConeLocalDerivatives(const Config& config, const vector3_t& forcesInTerrainFrame) {
  const auto& localForces = forcesInTerrainFrame;

  const auto F_x_square = localForces.x() * localForces.x();
  const auto F_y_square = localForces.y() * localForces.y();
  const auto F_tangent_square = F_x_square + F_y_square + config.regularization;
  const auto F_tangent_norm = std::sqrt(F_tangent_square);
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

  ConeLocalDerivatives coneLocalDerivatives{};
  coneLocalDerivatives.coneConstraint = config.frictionCoefficient * (localForces.z() + config.gripperForce) - F_tangent_norm;

  coneLocalDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
  coneLocalDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
  coneLocalDerivatives.dCone_dF(2) = config.frictionCoefficient;

  coneLocalDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config.regularization) / F_tangent_square_pow32 - config.hessianDiagonalShift;
  coneLocalDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneLocalDerivatives.d2Cone_dF2(0, 2) = 0.0;
  coneLocalDerivatives.d2Cone_dF2(1, 0) = coneLocalDerivatives.d2Cone_dF2(0, 1);
  coneLocalDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config.regularization) / F_tangent_square_pow32 - config.hessianDiagonalShift;
  coneLocalDerivatives.d2Cone_dF2(1, 2) = 0.0;
  coneLocalDerivatives.d2Cone_dF2(2, 0) = 0.0;
  coneLocalDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneLocalDerivatives.d2Cone_dF2(2, 2) = -config.hessianDiagonalShift;
  return coneLocalDerivatives;
}

ConeDerivatives frictionConeDerivatives(const Config& config, const vector3_t& forcesInTerrainFrame, const matrix3_t& t_R_w,
                                        const matrix3_t& w_R_b, const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) {
  const auto coneLocalDerivatives = frictionConeLocalDerivatives(config, forcesInTerrainFrame);

  // Local to body Derivatives
  matrix3_t dF_deuler = t_R_w * rotationBaseToOriginJacobian(eulerXYZ, forcesInBodyFrame);
  matrix3_t dF_du = t_R_w * w_R_b;

  ConeDerivatives coneDerivatives;
  // Zero order
  coneDerivatives.coneConstraint = coneLocalDerivatives.coneConstraint;

  // First order derivatives
  coneDerivatives.dCone_deuler.noalias() = coneLocalDerivatives.dCone_dF.transpose() * dF_deuler;
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * dF_du;

  // Second order derivatives
  matrix3_t tmp = coneLocalDerivatives.d2Cone_dF2 * dF_deuler;
  coneDerivatives.d2Cone_deuler2.noalias() = dF_deuler.transpose() * tmp;
  coneDerivatives.d2Cone_dudeuler.noalias() = dF_du.transpose() * tmp;

  tmp.noalias() = coneLocalDerivatives.d2Cone_dF2 * dF_du;
  coneDerivatives.d2Cone_du2.noalias() = dF_du.transpose() * tmp;

  return coneDerivatives;
}
}  // namespace friction_cone

}  // namespace switched_model
