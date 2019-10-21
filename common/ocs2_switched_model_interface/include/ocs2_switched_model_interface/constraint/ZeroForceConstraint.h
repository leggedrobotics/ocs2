#pragma once

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>

namespace switched_model {

class ZeroForceConstraint final : public ocs2::ConstraintTerm<24, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t STATE_DIM = 24;
  static constexpr size_t INPUT_DIM = 24;

  using BASE = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;
  using typename BASE::state_matrix_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::LinearApproximation_t;

  explicit ZeroForceConstraint(int legNumber) :
  BASE(ocs2::ConstraintOrder::Linear),
  legNumber_(legNumber) {}

  ZeroForceConstraint* clone() const override {
    return new ZeroForceConstraint(*this);
  }

  size_t getNumConstraints(scalar_t time) const override { return 3; };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    const scalar_t Fx = input(3 * legNumber_ + 0);
    const scalar_t Fy = input(3 * legNumber_ + 1);
    const scalar_t Fz = input(3 * legNumber_ + 2);

    scalar_array_t constraintValue;
    constraintValue.push_back(Fx);
    constraintValue.push_back(Fy);
    constraintValue.push_back(Fz);
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    LinearApproximation_t linearApproximation;
    for (int i =0; i<3; i++){
      linearApproximation.constraintValues.push_back( input(3 * legNumber_ + i) );
      linearApproximation.derivativeState.emplace_back( state_vector_t::Zero() );
      input_vector_t dcdu = state_vector_t::Zero();
      dcdu(3 * legNumber_ + i) = 1.0;
      linearApproximation.derivativeInput.push_back( dcdu );
    }

    return linearApproximation;
  }

 private:
  int legNumber_;

};

} // namespace switched_model