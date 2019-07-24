//
// Created by rgrandia on 28.06.19.
//

#ifndef OCS2_CTRL_FRICTIONCONECONSTRAINT_H
#define OCS2_CTRL_FRICTIONCONECONSTRAINT_H

#include <ocs2_core/constraint/ConstraintTerm.h>

namespace switched_model {

template <size_t STATE_DIM, size_t INPUT_DIM>
class FrictionConeConstraint final : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
 public:

  using BASE = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;
  using typename BASE::state_matrix_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;

  FrictionConeConstraint(double frictionCoefficient, double regularization, int legNumber)
      : BASE(ocs2::ConstraintOrder::Quadratic),
        frictionCoefficient_(frictionCoefficient),
        regularization_(regularization),
        legNumber_(legNumber) {}

  FrictionConeConstraint* clone() const override {
      return new FrictionConeConstraint(*this);
  }

  size_t getNumConstraints(scalar_t time) const override { return 1; };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    scalar_array_t constraintValue;
    constraintValue.emplace_back(frictionConeFunction(input));
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
      LinearApproximation_t linearApproximation;
      linearApproximation.constraintValues.emplace_back(frictionConeFunction(input));
      linearApproximation.derivativeState.emplace_back( state_vector_t::Zero() );
      linearApproximation.derivativeInput.emplace_back( frictionConeInputDerivative(input) );
      return linearApproximation;
  }

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
      QuadraticApproximation_t quadraticApproximation;
      quadraticApproximation.constraintValues.emplace_back(frictionConeFunction(input));
      quadraticApproximation.derivativeState.emplace_back( state_vector_t::Zero() );
      quadraticApproximation.derivativeInput.emplace_back( frictionConeInputDerivative(input) );
      quadraticApproximation.secondDerivativesState.emplace_back( state_matrix_t::Zero() );
      quadraticApproximation.secondDerivativesInput.emplace_back( frictionConeSecondDerivativeInput(input) );
      quadraticApproximation.derivativesInputState.emplace_back( input_state_matrix_t::Zero() );
      return quadraticApproximation;
  }

 private:
  scalar_t frictionConeFunction(const input_vector_t& input) const {
      const scalar_t Fx = input(3 * legNumber_ + 0);
      const scalar_t Fy = input(3 * legNumber_ + 1);
      const scalar_t Fz = input(3 * legNumber_ + 2);
      return Fz * sqrt(frictionCoefficient_ * frictionCoefficient_) - sqrt(Fx * Fx + Fy * Fy + regularization_);
  }

  input_vector_t frictionConeInputDerivative(const input_vector_t& input) const {
      input_vector_t dhdu;
      dhdu.setZero();
      const scalar_t Fx = input(3 * legNumber_ + 0);
      const scalar_t Fy = input(3 * legNumber_ + 1);
      const scalar_t Fz = input(3 * legNumber_ + 2);
      const scalar_t F_norm = sqrt(Fx*Fx+Fy*Fy+regularization_);
      dhdu(3 * legNumber_ + 0) = -Fx / F_norm;
      dhdu(3 * legNumber_ + 1) = -Fy / F_norm;
      dhdu(3 * legNumber_ + 2) = sqrt(frictionCoefficient_ * frictionCoefficient_);
      return dhdu;
  }

  input_matrix_t  frictionConeSecondDerivativeInput(const input_vector_t& input) const {
      input_matrix_t ddhdudu;
      ddhdudu.setZero();
      const scalar_t Fx = input(3 * legNumber_ + 0);
      const scalar_t Fy = input(3 * legNumber_ + 1);
      const scalar_t Fz = input(3 * legNumber_ + 2);
      const scalar_t F_norm2 = Fx*Fx+Fy*Fy+ regularization_;
      const scalar_t F_norm32 = pow(F_norm2, 1.5);
      ddhdudu(3 * legNumber_ + 0, 3 * legNumber_ + 0) = -(Fy*Fy  + regularization_) / F_norm32;
      ddhdudu(3 * legNumber_ + 0, 3 * legNumber_ + 1) = Fx * Fy / F_norm32;
      ddhdudu(3 * legNumber_ + 0, 3 * legNumber_ + 2) = 0.0;
      ddhdudu(3 * legNumber_ + 1, 3 * legNumber_ + 0) = Fx * Fy / F_norm32;
      ddhdudu(3 * legNumber_ + 1, 3 * legNumber_ + 1) = -(Fx*Fx + regularization_) / F_norm32;
      ddhdudu(3 * legNumber_ + 1, 3 * legNumber_ + 2) = 0.0;
      ddhdudu(3 * legNumber_ + 2, 3 * legNumber_ + 0) = 0.0;
      ddhdudu(3 * legNumber_ + 2, 3 * legNumber_ + 1) = 0.0;
      ddhdudu(3 * legNumber_ + 2, 3 * legNumber_ + 2) = 0.0;
      return ddhdudu;
  }

  double frictionCoefficient_;
  double regularization_;
  int legNumber_;
};

}  // namespace switched_model

#endif  // OCS2_CTRL_FRICTIONCONECONSTRAINT_H
