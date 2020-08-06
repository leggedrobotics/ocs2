#pragma once

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * frictionCoefficient_ * Fz - sqrt(Fx * Fx + Fy * Fy + regularization_) >= 0
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
  using LocalForces = vector3_t;

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

  scalar_t coneConstraint(const LocalForces& localForces) const;
  LocalForces computeLocalForces(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const LocalForces& localForces) const;
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
