//
// Created by rgrandia on 03.04.19.
//

#ifndef OCS2_TESTCONSTRAINTTERM_H
#define OCS2_TESTCONSTRAINTTERM_H

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>

class TestEmptyConstraint : public ocs2::ConstraintTerm<3, 2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::ConstraintTerm<3, 2>;

  TestEmptyConstraint() : BASE(ocs2::ConstraintOrder::None){};

  TestEmptyConstraint* clone() const override { return new TestEmptyConstraint(*this); };

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 0; };
  ocs2::scalar_array_t getValue(ocs2::scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    return ocs2::scalar_array_t();
  };

  LinearApproximation_t getLinearApproximation(ocs2::scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    return LinearApproximation_t();
  };
};

class TestLinearConstraint : public ocs2::ConstraintTerm<3, 2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::ConstraintTerm<3, 2>;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  TestLinearConstraint() : BASE(ocs2::ConstraintOrder::Linear){};

  TestLinearConstraint* clone() const override { return new TestLinearConstraint(*this); };

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 2; };

  ocs2::scalar_array_t getValue(ocs2::scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    ocs2::scalar_array_t constraintValues(2);
    constraintValues[0] = 1;
    constraintValues[1] = 2;
    return constraintValues;
  };

  LinearApproximation_t getLinearApproximation(ocs2::scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues = getValue(time, state, input);
    linearApproximation.derivativeState.emplace_back(state_vector_t::Zero());
    linearApproximation.derivativeState.emplace_back(state_vector_t::Ones());
    linearApproximation.derivativeInput.emplace_back(input_vector_t::Zero());
    linearApproximation.derivativeInput.emplace_back(input_vector_t::Ones());
    return linearApproximation;
  };

  QuadraticApproximation_t getQuadraticApproximation(ocs2::scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override {
    QuadraticApproximation_t quadraticApproximation;
    quadraticApproximation.constraintValues = getValue(time, state, input);
    quadraticApproximation.derivativeState.emplace_back(state_vector_t::Ones());
    quadraticApproximation.derivativeInput.emplace_back(input_vector_t::Ones());
    quadraticApproximation.secondDerivativesState.emplace_back(state_matrix_t::Ones());
    quadraticApproximation.secondDerivativesInput.emplace_back(input_matrix_t::Ones());
    quadraticApproximation.derivativesInputState.emplace_back(input_state_matrix_t::Ones());
    return quadraticApproximation;
  };
};

#endif  // OCS2_TESTCONSTRAINTTERM_H
