#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

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
class FrictionConeConstraint final : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
   * Constructor
   * @param [in] config : Friction model settings.
   * @param [in] legNumber : leg index in {0, 1, 2, 3}.
   */
  FrictionConeConstraint(Config config, int legNumber);

  ~FrictionConeConstraint() override = default;

  FrictionConeConstraint* clone() const override { return new FrictionConeConstraint(*this); }

  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

  size_t getNumConstraints(scalar_t time) const override { return 1; };

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const override;

  VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const vector_t& state,
                                                           const vector_t& input) const override;

  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const vector_t& input) const override;

 private:
  struct LocalForceDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    matrix3_t dF_deuler;  // derivative local force w.r.t. body euler angles
    matrix3_t dF_du;      // derivative local force w.r.t. forces in body frame
  };

  struct ConeLocalDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_dF;    // derivate w.r.t local force
    matrix3_t d2Cone_dF2;  // second derivative w.r.t local force
  };

  struct ConeDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_deuler;
    vector3_t dCone_du;
    matrix3_t d2Cone_du2;
    matrix3_t d2Cone_dudeuler;
    matrix3_t d2Cone_deuler2;
  };

  FrictionConeConstraint(const FrictionConeConstraint& other) = default;
  vector_t coneConstraint(const vector3_t& localForces) const;
  vector3_t computeLocalForces(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;

  matrix_t frictionConeStateDerivative(const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeDerivativesInputState(const ConeDerivatives& coneDerivatives) const;

  const Config config_;
  const int legNumber_;
  matrix3_t t_R_w;  // rotation world to terrain
};

}  // namespace switched_model
