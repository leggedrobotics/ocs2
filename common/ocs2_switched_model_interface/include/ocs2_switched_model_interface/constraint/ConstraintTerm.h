//
// Created by rgrandia on 28.03.19.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_switched_model_interface/Dimensions.h>

namespace ocs2 {

enum class ConstraintOrder { None, Linear, Quadratic };

template <size_t STATE_DIM, size_t INPUT_DIM>
struct LinearConstraintApproximation {
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;

  scalar_array_t constraintValues;
  state_vector_array_t derivativeState;
  input_vector_array_t derivativeInput;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
struct LinearConstraintApproximationAsMatrices {
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;

  constraint1_vector_t constraintValues;
  constraint1_state_matrix_t derivativeState;
  constraint1_input_matrix_t derivativeInput;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
struct QuadraticConstraintApproximation {
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;

  scalar_array_t constraintValues;
  state_vector_array_t derivativeState;
  input_vector_array_t derivativeInput;
  state_matrix_array_t secondDerivativesState;
  input_matrix_array_t secondDerivativesInput;
  input_state_matrix_array_t derivativesInputState;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintTerm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;

  using LinearApproximation_t = LinearConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using QuadraticApproximation_t = QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;

  explicit ConstraintTerm(ConstraintOrder oder) : order_(oder), active_(true){};
  virtual ~ConstraintTerm() = default;
  virtual ConstraintTerm* clone() const = 0;

  // Modifiers
  void setActivity(bool activity) { active_ = activity; };

  // Observers
  constexpr ConstraintOrder getOrder() const { return order_; };
  bool isActive() const { return active_; };

  // Evaluate the constraint
  virtual size_t getNumConstraints(scalar_t time) const = 0;
  virtual scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const = 0;
  virtual LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const {
    throw std::runtime_error("[ConstraintTerm] Linear approximation not implemented");
  };
  virtual QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                             const input_vector_t& input) const {
    throw std::runtime_error("[ConstraintTerm] Quadratic approximation not implemented");
  };

 private:
  ConstraintOrder order_;
  bool active_;
};

};  // namespace ocs2
