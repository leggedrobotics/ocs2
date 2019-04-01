//
// Created by rgrandia on 06.02.19.
//

#ifndef PENALTYBASE_OCS2_H
#define PENALTYBASE_OCS2_H

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

    /*
     *   Implements the cost penalty for the inequality constraint
     *   h_i(x, u) <= 0, i in [1,..,M]
     *
     *   penalty = sum_{i=1}^{M} p(h_i(x, u))
     *
     *   The scalar penalty function p() and its derivatives are to be implemented in the derived class
     */
    template<size_t STATE_DIM, size_t INPUT_DIM>
    class PenaltyBase {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
        typedef typename DIMENSIONS::scalar_t                   scalar_t;
        typedef typename DIMENSIONS::scalar_array_t             scalar_array_t;
        typedef typename DIMENSIONS::state_vector_t             state_vector_t;
        typedef typename DIMENSIONS::state_vector_array_t       state_vector_array_t;
        typedef typename DIMENSIONS::input_vector_t             input_vector_t;
        typedef typename DIMENSIONS::input_vector_array_t       input_vector_array_t;
        typedef typename DIMENSIONS::state_matrix_t             state_matrix_t;
        typedef typename DIMENSIONS::state_matrix_array_t       state_matrix_array_t;
        typedef typename DIMENSIONS::input_matrix_t             input_matrix_t;
        typedef typename DIMENSIONS::input_matrix_array_t       input_matrix_array_t;
        typedef typename DIMENSIONS::input_state_matrix_t       input_state_matrix_t;
        typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;

        PenaltyBase() = default;

        virtual ~PenaltyBase() = default;

        scalar_t getPenaltyCost(size_t numInequalityConstraints, const scalar_array_t &h) const;

        void getPenaltyCostDerivativeState(size_t numInequalityConstraints,
                                           const scalar_array_t &h, const state_vector_array_t &dhdx,
                                           state_vector_t &penaltyDerivativeState) const;

        void getPenaltyCostDerivativeInput(size_t numInequalityConstraints, const scalar_array_t &h,
                                           const input_vector_array_t &dhdu,
                                           input_vector_t &penaltyDerivativeInput) const;

        void getPenaltyCostSecondDerivativeState(size_t numInequalityConstraints, const scalar_array_t &h,
                                                 const state_vector_array_t &dhdx,
                                                 const state_matrix_array_t &ddhdxdx,
                                                 state_matrix_t &penaltySecondDerivativeState) const;

        void getPenaltyCostSecondDerivativeInput(size_t numInequalityConstraints, const scalar_array_t &h,
                                                 const input_vector_array_t &dhdu,
                                                 const input_matrix_array_t &ddhdudu,
                                                 input_matrix_t &penaltySecondDerivativeInput) const;

        void getPenaltyCostDerivativeInputState(size_t numInequalityConstraints, const scalar_array_t &h,
                                                const state_vector_array_t &dhdx,
                                                const input_vector_array_t &dhdu,
                                                const input_state_matrix_array_t &ddhdudx,
                                                input_state_matrix_t &penaltyDerivativeInputState) const;

        scalar_t getConstraintViolationSquaredNorm(size_t numInequalityConstraints, const scalar_array_t &h) const;

    private:
        virtual scalar_t getPenaltyFunctionValue(scalar_t h) const = 0;

        virtual scalar_t getPenaltyFunctionDerivative(scalar_t h) const = 0;

        virtual scalar_t getPenaltyFunctionHessian(scalar_t h) const = 0;
    };
}

#include "implementation/PenaltyBase.h"

#endif //PENALTYBASE_OCS2_H
