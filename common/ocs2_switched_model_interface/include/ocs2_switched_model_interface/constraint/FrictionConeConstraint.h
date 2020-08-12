#pragma once

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
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
class FrictionConeConstraint final : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  /**
   * Constructor
   * @param frictionCoefficient : Friction coefficient (>0).
   * @param regularization : (>0), see class documentation.
   * @param legNumber : leg index in {0, 1, 2, 3}.
   * @param gripperForce : Gripper force in normal direction.
   */
  FrictionConeConstraint(scalar_t frictionCoefficient, scalar_t regularization, int legNumber, scalar_t gripperForce = 0.0);

  ~FrictionConeConstraint() override = default;

  FrictionConeConstraint* clone() const override { return new FrictionConeConstraint(*this); }

  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

  size_t getNumConstraints(scalar_t time) const override { return 1; };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override;

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override;

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override;

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

  scalar_t coneConstraint(const vector3_t& localForces) const;
  vector3_t computeLocalForces(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;

  state_vector_t frictionConeStateDerivative(const ConeDerivatives& coneDerivatives) const;
  input_vector_t frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const;
  input_matrix_t frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const;
  state_matrix_t frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const;
  input_state_matrix_t frictionConeDerivativesInputState(const ConeDerivatives& coneDerivatives) const;

  scalar_t frictionCoefficient_;
  scalar_t regularization_;
  scalar_t gripperForce_;
  int legNumber_;
  matrix3_t t_R_w;  // rotation world to terrain
};

}  // namespace switched_model
