//
// Created by rgrandia on 03.04.19.
//

#pragma once

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
size_t ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;

  // Accumulate number of constraints for each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      numConstraints += constraintTerm->getNumConstraints(time);
    }
  }

  return numConstraints;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
typename ConstraintCollectionView<STATE_DIM, INPUT_DIM>::scalar_array_t ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getValue(
    scalar_t time, const state_vector_t& state, const input_vector_t& input) const {
  scalar_array_t constraintValues;
  constraintValues.reserve(getNumConstraints(time));

  // Append vectors of constraint values from each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      scalar_array_t constraintTermValues = constraintTerm->getValue(time, state, input);
      appendVectorToVectorByMoving(constraintValues, constraintTermValues);
    }
  }

  return constraintValues;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
typename ConstraintCollectionView<STATE_DIM, INPUT_DIM>::LinearApproximation_t
ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getLinearApproximation(scalar_t time, const state_vector_t& state,
                                                                       const input_vector_t& input) const {
  const auto numConstraints = getNumConstraints(time);
  LinearApproximation_t linearApproximation;
  linearApproximation.constraintValues.reserve(numConstraints);
  linearApproximation.derivativeState.reserve(numConstraints);
  linearApproximation.derivativeInput.reserve(numConstraints);

  // Append linearApproximation each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      auto constraintTermApproximation = constraintTerm->getLinearApproximation(time, state, input);
      appendVectorToVectorByMoving(linearApproximation.constraintValues, constraintTermApproximation.constraintValues);
      appendVectorToVectorByMoving(linearApproximation.derivativeState, constraintTermApproximation.derivativeState);
      appendVectorToVectorByMoving(linearApproximation.derivativeInput, constraintTermApproximation.derivativeInput);
    }
  }

  return linearApproximation;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename ConstraintCollectionView<STATE_DIM, INPUT_DIM>::QuadraticApproximation_t
ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                                          const input_vector_t& input) const {
  const auto numConstraints = getNumConstraints(time);
  QuadraticApproximation_t quadraticApproximation;

  quadraticApproximation.constraintValues.reserve(numConstraints);
  quadraticApproximation.derivativeState.reserve(numConstraints);
  quadraticApproximation.derivativeInput.reserve(numConstraints);
  quadraticApproximation.secondDerivativesState.reserve(numConstraints);
  quadraticApproximation.secondDerivativesInput.reserve(numConstraints);
  quadraticApproximation.derivativesInputState.reserve(numConstraints);

  // Append linearApproximation each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      auto constraintTermApproximation = constraintTerm->getQuadraticApproximation(time, state, input);
      appendVectorToVectorByMoving(quadraticApproximation.constraintValues, constraintTermApproximation.constraintValues);
      appendVectorToVectorByMoving(quadraticApproximation.derivativeState, constraintTermApproximation.derivativeState);
      appendVectorToVectorByMoving(quadraticApproximation.derivativeInput, constraintTermApproximation.derivativeInput);
      appendVectorToVectorByMoving(quadraticApproximation.secondDerivativesState, constraintTermApproximation.secondDerivativesState);
      appendVectorToVectorByMoving(quadraticApproximation.secondDerivativesInput, constraintTermApproximation.secondDerivativesInput);
      appendVectorToVectorByMoving(quadraticApproximation.derivativesInputState, constraintTermApproximation.derivativesInputState);
    }
  }

  return quadraticApproximation;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
Eigen::VectorXd ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getValueAsVector(scalar_t time, const state_vector_t& state,
                                                                                 const input_vector_t& input) const {
  Eigen::VectorXd constraintVector(getNumConstraints(time));
  int nextFreeIndex = 0;

  // Append vectors of constraint values from each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      scalar_array_t constraintTermValues = constraintTerm->getValue(time, state, input);

      // Add constraint to next free index of Eigen::VectorXd
      for (const scalar_t& v : constraintTermValues) {
        constraintVector[nextFreeIndex] = v;
        nextFreeIndex++;
      }
    }
  }

  return constraintVector;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
typename ConstraintCollectionView<STATE_DIM, INPUT_DIM>::LinearApproximationAsMatrices_t
ConstraintCollectionView<STATE_DIM, INPUT_DIM>::getLinearApproximationAsMatrices(scalar_t time, const state_vector_t& state,
                                                                                 const input_vector_t& input) const {
  LinearApproximationAsMatrices_t linearApproximationAsMatrices;
  int nextFreeIndex = 0;

  // Append linearApproximation each constraintTerm
  for (const auto& constraintTerm : constraintTerms_) {
    if (constraintTerm->isActive()) {
      auto constraintTermApproximation = constraintTerm->getLinearApproximation(time, state, input);

      // Add constraint to next free index of eigen vector vector
      for (int i = 0; i < constraintTermApproximation.constraintValues.size(); i++) {
        if (nextFreeIndex >= linearApproximationAsMatrices.constraintValues.rows()) {
          throw std::runtime_error(
              "[ConstraintCollectionView::getLinearApproximationAsMatrices] More constraints than rows available in the approximation "
              "matrices");
        }

        linearApproximationAsMatrices.constraintValues[nextFreeIndex] = constraintTermApproximation.constraintValues[i];
        linearApproximationAsMatrices.derivativeState.row(nextFreeIndex) = constraintTermApproximation.derivativeState[i];
        linearApproximationAsMatrices.derivativeInput.row(nextFreeIndex) = constraintTermApproximation.derivativeInput[i];
        nextFreeIndex++;
      }
    }
  }

  return linearApproximationAsMatrices;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename T, typename Allocator>
inline void ConstraintCollectionView<STATE_DIM, INPUT_DIM>::appendVectorToVectorByMoving(std::vector<T, Allocator>& v1,
                                                                                         std::vector<T, Allocator>& v2) {
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

}  // namespace ocs2