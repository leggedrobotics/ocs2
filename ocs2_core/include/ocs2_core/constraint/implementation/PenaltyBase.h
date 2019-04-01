//
// Created by rgrandia on 06.02.19.
//

#include <ocs2_core/constraint/PenaltyBase.h>

namespace ocs2 {

    template<size_t STATE_DIM, size_t INPUT_DIM>
    typename ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_t
    PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCost(size_t numInequalityConstraints,
                                                      const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h) const {
        scalar_t penalty = 0;
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penalty += getPenaltyFunctionValue(h[i]);
        }
        return penalty;
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeState(size_t numInequalityConstraints,
                                                                          const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h,
                                                                          const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_vector_array_t &dhdx,
                                                                          ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_vector_t &penaltyDerivativeState) const {
        penaltyDerivativeState.setZero();
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penaltyDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdx[i];
        }
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInput(size_t numInequalityConstraints,
                                                                          const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h,
                                                                          const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_vector_array_t &dhdu,
                                                                          ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_vector_t &penaltyDerivativeInput) const {
        penaltyDerivativeInput.setZero();
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penaltyDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdu[i];
        }
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeState(size_t numInequalityConstraints,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_vector_array_t &dhdx,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_matrix_array_t &ddhdxdx,
                                                                                ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_matrix_t &penaltySecondDerivativeState) const {
        penaltySecondDerivativeState.setZero();
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penaltySecondDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdxdx[i];
            penaltySecondDerivativeState.noalias() += getPenaltyFunctionHessian(h[i]) * dhdx[i] * dhdx[i].transpose();
        }
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeInput(size_t numInequalityConstraints,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_vector_array_t &dhdu,
                                                                                const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_matrix_array_t &ddhdudu,
                                                                                ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_matrix_t &penaltySecondDerivativeInput) const {
        penaltySecondDerivativeInput.setZero();
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penaltySecondDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudu[i];
            penaltySecondDerivativeInput.noalias() += getPenaltyFunctionHessian(h[i]) * dhdu[i] * dhdu[i].transpose();
        }
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInputState(size_t numInequalityConstraints,
                                                                               const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h,
                                                                               const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::state_vector_array_t &dhdx,
                                                                               const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_vector_array_t &dhdu,
                                                                               const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_state_matrix_array_t &ddhdudx,
                                                                               ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::input_state_matrix_t &penaltyDerivativeInputState) const {
        penaltyDerivativeInputState.setZero();
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            penaltyDerivativeInputState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudx[i];
            penaltyDerivativeInputState.noalias() += getPenaltyFunctionHessian(h[i]) * dhdu[i] * dhdx[i].transpose();
        }
    };

    template<size_t STATE_DIM, size_t INPUT_DIM>
    typename ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_t
    PenaltyBase<STATE_DIM, INPUT_DIM>::getConstraintViolationSquaredNorm(size_t numInequalityConstraints,
                                                      const ocs2::PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_array_t &h) const {
        scalar_t squaredViolation = 0;
        for (size_t i = 0; i < numInequalityConstraints; i++) {
            scalar_t violation = std::max(0.0, -h[i]);
            squaredViolation += violation*violation;
        }
        return squaredViolation;
    };

}