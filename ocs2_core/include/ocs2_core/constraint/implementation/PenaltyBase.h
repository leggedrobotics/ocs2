

#include <ocs2_core/constraint/PenaltyBase.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCost(const scalar_array_t& h, scalar_t& penalty) const {
  penalty = 0;
  for (const scalar_t& hi : h) {
    penalty += getPenaltyFunctionValue(hi);
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeState(const scalar_array_t& h, const state_vector_array_t& dhdx,
                                                                      state_vector_t& penaltyDerivativeState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdx[i];
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInput(const scalar_array_t& h, const input_vector_array_t& dhdu,
                                                                      input_vector_t& penaltyDerivativeInput) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeInput.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdu[i];
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const state_vector_array_t& dhdx,
                                                                            const state_matrix_array_t& ddhdxdx,
                                                                            state_matrix_t& penaltySecondDerivativeState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != ddhdxdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltySecondDerivativeState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdxdx[i];
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdx[i] * dhdx[i].transpose();
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const input_vector_array_t& dhdu,
                                                                            const input_matrix_array_t& ddhdudu,
                                                                            input_matrix_t& penaltySecondDerivativeInput) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size() || numInequalityConstraints != ddhdudu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltySecondDerivativeInput.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudu[i];
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdu[i].transpose();
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInputState(const scalar_array_t& h, const state_vector_array_t& dhdx,
                                                                           const input_vector_array_t& dhdu,
                                                                           const input_state_matrix_array_t& ddhdudx,
                                                                           input_state_matrix_t& penaltyDerivativeInputState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeInputState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudx[i];
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdx[i].transpose();
  }
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getConstraintViolationSquaredNorm(const scalar_array_t& h, scalar_t& squaredViolation) const {
  squaredViolation = 0;
  for (const scalar_t& hi : h) {
    scalar_t violation = std::max(0.0, -hi);
    squaredViolation += violation * violation;
  }
};

}  // namespace ocs2
