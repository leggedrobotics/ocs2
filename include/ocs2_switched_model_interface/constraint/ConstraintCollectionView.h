//
// Created by rgrandia on 03.04.19.
//

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>
#include "ocs2_core/Dimensions.h"

#include "ocs2_switched_model_interface/constraint/ConstraintTerm.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintCollectionView {
 public:
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;

  using ConstraintTerm_t = ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using LinearApproximation_t = LinearConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using LinearApproximationAsMatrices_t = LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticApproximation_t = QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;

  explicit ConstraintCollectionView(const std::vector<ConstraintTerm_t const*>& constraintTerms) : constraintTerms_(constraintTerms){};

  size_t getNumConstraints(scalar_t time) const;

  // Get as std::vectors
  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;

  // Get as Eigen
  Eigen::VectorXd getValueAsVector(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  LinearApproximationAsMatrices_t getLinearApproximationAsMatrices(scalar_t time, const state_vector_t& state,
                                                                   const input_vector_t& input) const;

 private:
  template <typename T, typename Allocator>
  static inline void appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>& v2);

  const std::vector<ConstraintTerm_t const*>& constraintTerms_;
};

};  // namespace ocs2

#include "implementation/ConstraintCollectionView.h"
