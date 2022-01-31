#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

namespace switched_model {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * frictionCoefficient * (Fz + gripperForce) - sqrt(Fx * Fx + Fy * Fy + regularization) >= 0
 *
 * The gripper force shifts the origin of the friction cone down in z-direction by the amount of gripping force available. This makes it
 * possible to produce tangential forces without applying a regular normal force on that foot, or to "pull" on the foot with magnitude up to
 * the gripping force.
 *
 * The regularization prevents the constraint gradient / hessian to go to infinity when Fx = Fz = 0. It also creates a parabolic safety
 * margin to the friction cone. For example: when Fx = Fy = 0 the constraint zero-crossing will be at Fz = 1/frictionCoefficient *
 * sqrt(regularization) instead of Fz = 0
 *
 */
namespace friction_cone {

/**
 * frictionCoefficient: The coefficient of friction.
 * regularization: A positive number to regulize the friction constraint. refer to the FrictionConeConstraint documentation.
 * gripperForce: Gripper force in normal direction.
 * hessianDiagonalShift: The Hessian shift to assure a strictly-convex quadratic constraint approximation.
 */
struct Config {
  explicit Config(scalar_t frictionCoefficientParam = 0.7, scalar_t regularizationParam = 25.0, scalar_t gripperForceParam = 0.0,
                  scalar_t hessianDiagonalShiftParam = 1e-6)
      : frictionCoefficient(frictionCoefficientParam),
        regularization(regularizationParam),
        gripperForce(gripperForceParam),
        hessianDiagonalShift(hessianDiagonalShiftParam) {
    assert(frictionCoefficient > 0.0);
    assert(regularization > 0.0);
    assert(hessianDiagonalShift >= 0.0);
  }

  scalar_t frictionCoefficient;
  scalar_t regularization;
  scalar_t gripperForce;
  scalar_t hessianDiagonalShift;
};

/**
 * Computes the friction cone constraint
 *
 * @param config : friction cone configuration
 * @param forcesInTerrainFrame : forces defined in the terrain frame. The z component is aligned with the normal direction.
 * @return scalar constraint value h(t,x,u) >= 0
 */
scalar_t frictionConeConstraint(const Config& config, const vector3_t& forcesInTerrainFrame);

/**
 * Zero, first, and second order derivative of the friction cone constraint w.r.t forces in the terrain
 */
struct ConeLocalDerivatives {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// constraint value
  scalar_t coneConstraint;

  /// first derivative w.r.t force in the terrain frame
  vector3_t dCone_dF;

  /// second derivative w.r.t force in the terrain frame
  matrix3_t d2Cone_dF2;
};

/**
 * Compute the derivatives of the friction cone constraint w.r.t. the local forces
 *
 * @param config : friction cone configuration
 * @param forcesInTerrainFrame : forces defined in the terrain frame. The z component is aligned with the normal direction.
 * @return zero, first, and second derivative of the friction cone.
 */
ConeLocalDerivatives frictionConeLocalDerivatives(const Config& config, const vector3_t& forcesInTerrainFrame);

/**
 * Zero, first, and second order derivative of the friction cone constraint w.r.t forces in the body frame
 */
struct ConeDerivatives {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// constraint value
  scalar_t coneConstraint;

  /// first derivative w.r.t body orientation (EulerXYZ)
  vector3_t dCone_deuler;

  /// first derivative w.r.t forces in body frame
  vector3_t dCone_du;

  /// second derivative w.r.t body orientation (EulerXYZ)
  matrix3_t d2Cone_deuler2;

  /// second derivative w.r.t forces in body frame
  matrix3_t d2Cone_du2;

  /// second derivative coupling terms {forces in body frame} x {body orientation (EulerXYZ)}
  matrix3_t d2Cone_dudeuler;
};

/**
 * Computes the derivatives of the friction cone constraint w.r.t. forces in body frame and the body orientation (eulerXYZ).
 *
 * @param config : friction cone configuration
 * @param forcesInTerrainFrame : forces defined in the terrain frame. The z component is aligned with the normal direction.
 * @param t_R_w : rotation matrix world to terrain
 * @param w_R_b : rotation matrix body to world
 * @param eulerXYZ : current body orientation
 * @param forcesInBodyFrame : forces in body frame
 * @return zero, first, and second derivative of the friction cone.
 */
ConeDerivatives frictionConeDerivatives(const Config& config, const vector3_t& forcesInTerrainFrame, const matrix3_t& t_R_w,
                                        const matrix3_t& w_R_b, const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame);

}  // namespace friction_cone

}  // namespace switched_model
