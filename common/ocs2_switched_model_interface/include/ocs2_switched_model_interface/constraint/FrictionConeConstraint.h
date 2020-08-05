#pragma once

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * frictionCoefficient_ * Fz - sqrt(Fx * Fx + Fy * Fy + regularization_)
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

  FrictionConeConstraint(scalar_t frictionCoefficient, scalar_t regularization, int legNumber, scalar_t gripperForce = 0.0)
      : BASE(ocs2::ConstraintOrder::Quadratic),
        frictionCoefficient_(frictionCoefficient),
        regularization_(regularization),
        gripperForce_(gripperForce),
        legNumber_(legNumber),
        t_R_w(matrix3_t::Identity()) {}

  FrictionConeConstraint* clone() const override { return new FrictionConeConstraint(*this); }

  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
    t_R_w = orientationWorldToTerrainFromSurfaceNormalInWorld(surfaceNormalInWorld);
  }

  size_t getNumConstraints(scalar_t time) const override { return 1; };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    const vector3_t eulerXYZ = getOrientation(getComPose(state));
    const vector3_t forcesInBodyFrame = input.template segment<3>(3 * legNumber_);

    const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);

    scalar_array_t constraintValue;
    constraintValue.emplace_back(coneConstraint(localForce));
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    const vector3_t eulerXYZ = getOrientation(getComPose(state));
    const vector3_t forcesInBodyFrame = input.template segment<3>(3 * legNumber_);

    const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);
    const auto localForceDerivatives = computeLocalForceDerivatives(eulerXYZ, forcesInBodyFrame);
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues.emplace_back(coneConstraint(localForce));
    linearApproximation.derivativeState.emplace_back(frictionConeStateDerivative(coneDerivatives));
    linearApproximation.derivativeInput.emplace_back(frictionConeInputDerivative(coneDerivatives));
    return linearApproximation;
  }

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override {
    const vector3_t eulerXYZ = getOrientation(getComPose(state));
    const vector3_t forcesInBodyFrame = input.template segment<3>(3 * legNumber_);

    const auto localForce = computeLocalForces(eulerXYZ, forcesInBodyFrame);
    const auto localForceDerivatives = computeLocalForceDerivatives(eulerXYZ, forcesInBodyFrame);
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

    QuadraticApproximation_t quadraticApproximation;
    quadraticApproximation.constraintValues.emplace_back(coneConstraint(localForce));
    quadraticApproximation.derivativeState.emplace_back(frictionConeStateDerivative(coneDerivatives));
    quadraticApproximation.derivativeInput.emplace_back(frictionConeInputDerivative(coneDerivatives));
    quadraticApproximation.secondDerivativesState.emplace_back(frictionConeSecondDerivativeState(coneDerivatives));
    quadraticApproximation.secondDerivativesInput.emplace_back(frictionConeSecondDerivativeInput(coneDerivatives));
    quadraticApproximation.derivativesInputState.emplace_back(frictionConeDerivativesInputState(coneDerivatives));
    return quadraticApproximation;
  }

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
    vector3_t dCone_deuler;
    vector3_t dCone_du;
    matrix3_t d2Cone_du2;
    matrix3_t d2Cone_dudeuler;
    matrix3_t d2Cone_deuler2;
  };

  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const {
    LocalForceDerivatives localForceDerivatives{};
    localForceDerivatives.dF_deuler = t_R_w * rotationBaseToOriginJacobian(eulerXYZ, forcesInBodyFrame);
    localForceDerivatives.dF_du = t_R_w * rotationMatrixBaseToOrigin(eulerXYZ);
    return localForceDerivatives;
  }

  LocalForces computeLocalForces(const vector3_t& eulerXYZ, const vector3_t& forcesInBodyFrame) const {
    matrix3_t t_R_b = t_R_w * rotationMatrixBaseToOrigin(eulerXYZ);
    return t_R_b * forcesInBodyFrame;
  }

  ConeLocalDerivatives computeConeLocalDerivatives(const LocalForces& localForces) const {
    const auto F_tangent = localForces.x() * localForces.x() + localForces.y() * localForces.y() + regularization_;
    const auto F_norm = sqrt(F_tangent);
    const auto F_norm32 = F_tangent * F_norm;

    ConeLocalDerivatives coneDerivatives{};
    coneDerivatives.dCone_dF(0) = -localForces.x() / F_norm;
    coneDerivatives.dCone_dF(1) = -localForces.y() / F_norm;
    coneDerivatives.dCone_dF(2) = frictionCoefficient_;

    coneDerivatives.d2Cone_dF2(0, 0) = -(localForces.y() * localForces.y() + regularization_) / F_norm32;
    coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_norm32;
    coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
    coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
    coneDerivatives.d2Cone_dF2(1, 1) = -(localForces.x() * localForces.x() + regularization_) / F_norm32;
    coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

    return coneDerivatives;
  }

  scalar_t coneConstraint(const LocalForces& localForces) const {
    const auto F_tangent = localForces.x() * localForces.x() + localForces.y() * localForces.y() + regularization_;
    const auto F_norm = sqrt(F_tangent);

    return frictionCoefficient_ * (localForces.z() + gripperForce_) - F_norm;
  }

  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const {
    ConeDerivatives coneDerivatives;
    // First order derivatives
    coneDerivatives.dCone_deuler = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_deuler;
    coneDerivatives.dCone_du = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

    // Second order derivatives
    coneDerivatives.d2Cone_du2 = localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;
    coneDerivatives.d2Cone_deuler2 =
        localForceDerivatives.dF_deuler.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_deuler;
    coneDerivatives.d2Cone_dudeuler =
        localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_deuler;

    return coneDerivatives;
  }

  state_vector_t frictionConeStateDerivative(const ConeDerivatives& coneDerivatives) const {
    state_vector_t dhdx;
    dhdx.setZero();
    dhdx.segment<3>(0) = coneDerivatives.dCone_deuler;
    return dhdx;
  }

  input_vector_t frictionConeInputDerivative(const ConeDerivatives& coneDerivatives) const {
    input_vector_t dhdu;
    dhdu.setZero();
    dhdu.segment<3>(3 * legNumber_) = coneDerivatives.dCone_du;
    return dhdu;
  }

  input_matrix_t frictionConeSecondDerivativeInput(const ConeDerivatives& coneDerivatives) const {
    input_matrix_t ddhdudu;
    ddhdudu.setZero();
    ddhdudu.block<3, 3>(3 * legNumber_, 3 * legNumber_) = coneDerivatives.d2Cone_du2;
    return ddhdudu;
  }

  state_matrix_t frictionConeSecondDerivativeState(const ConeDerivatives& coneDerivatives) const {
    input_matrix_t ddhdxdx;
    ddhdxdx.setZero();
    ddhdxdx.block<3, 3>(0, 0) = coneDerivatives.d2Cone_deuler2;
    return ddhdxdx;
  }

  input_state_matrix_t frictionConeDerivativesInputState(const ConeDerivatives& coneDerivatives) const {
    input_state_matrix_t ddhdudx;
    ddhdudx.setZero();
    ddhdudx.block<3, 3>(3 * legNumber_, 0) = coneDerivatives.d2Cone_dudeuler;
    return ddhdudx;
  }

  scalar_t frictionCoefficient_;
  scalar_t regularization_;
  scalar_t gripperForce_;
  int legNumber_;
  matrix3_t t_R_w;  // rotation world to terrain
};

}  // namespace switched_model
