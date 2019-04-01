//
// Created by rgrandia on 06.02.19.
//

#ifndef RELAXEDBARRIERPENALTY_OCS2_H
#define RELAXEDBARRIERPENALTY_OCS2_H

#include "PenaltyBase.h"

namespace ocs2 {

    /*
     *  Implements the relaxed barrier function from
     *  h >= 0
     *
     *  with mu, delta > 0
     *
     *  -mu * log(h)    if      h > delta
     *  -mu * log(delta) + 0.5*( (h-2*delta)/delta )^2 - 0.5     h <= delta
     */
    template<size_t STATE_DIM, size_t INPUT_DIM>
    class RelaxedBarrierPenalty final : public PenaltyBase<STATE_DIM, INPUT_DIM> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_t scalar_t;

        RelaxedBarrierPenalty(scalar_t mu, scalar_t delta) : mu_(mu), delta_(delta) {};
        virtual ~RelaxedBarrierPenalty() = default;

    private:
        scalar_t mu_ = 1e-1;
        scalar_t delta_ = 1e-3;

        virtual scalar_t getPenaltyFunctionValue(scalar_t h) const override {
            if (h > delta_){
                return -mu_ * log(h);
            } else {
                return mu_ * (-log(delta_) + scalar_t(0.5)* pow((h - 2.0*delta_) / delta_ , 2) - scalar_t(0.5));
            };
        };

        virtual scalar_t getPenaltyFunctionDerivative(scalar_t h) const override {
            if (h > delta_){
                return -mu_ / h;
            } else {
                return mu_ * ( (h - 2*delta_) / (delta_ * delta_) );
            };
        };

        virtual scalar_t getPenaltyFunctionHessian(scalar_t h) const override {
            if (h > delta_){
                return mu_ / (h*h);
            } else {
                return mu_ / (delta_ * delta_);
            };
        };
    };
}

#endif // RELAXEDBARRIERPENALTY_OCS2_H
