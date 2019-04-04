//
// Created by rgrandia on 06.02.19.
//

#ifndef RELAXEDBARRIERPENALTY_OCS2_H
#define RELAXEDBARRIERPENALTY_OCS2_H

#include "PenaltyBase.h"

namespace ocs2 {

/**
 *  Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$
 *
 *   \f[
 *   p(h)=\left\{
 *               \begin{array}{ll}
 *                 -\mu \ln(h) & if \quad  h > \delta, \\
 *                 -\mu \ln(\delta) + \frac{1}{2} \left( \left( \frac{h-2\delta}{\delta} \right)^2 - 1 \right) & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 *  where \f$ \mu \geq 0 \f$, and \f$ \delta \geq 0 \f$ are user defined parameters.
 *
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

  /**
  * Compute the penalty value at a certain constraint value.
  *
  * @param [in] h: Constraint value.
  * @return penalty cost.
  */
  virtual scalar_t getPenaltyFunctionValue(scalar_t h) const override {
    if (h > delta_) {
      return -mu_ * log(h);
    } else {
      return mu_ * (-log(delta_) + scalar_t(0.5) * pow((h - 2.0 * delta_) / delta_, 2) - scalar_t(0.5));
    };
  };

  /**
  * Compute the penalty derivative at a certain constraint value.
  *
  * @param [in] h: Constraint value.
  * @return penalty derivative with respect to constraint value.
  */
  virtual scalar_t getPenaltyFunctionDerivative(scalar_t h) const override {
    if (h > delta_) {
      return -mu_ / h;
    } else {
      return mu_ * ((h - 2 * delta_) / (delta_ * delta_));
    };
  };

  /**
  * Compute the penalty second derivative at a certain constraint value.
  *
  * @param [in] h: Constraint value.
  * @return penalty second derivative with respect to constraint value.
  */
  virtual scalar_t getPenaltyFunctionHessian(scalar_t h) const override {
    if (h > delta_) {
      return mu_ / (h * h);
    } else {
      return mu_ / (delta_ * delta_);
    };
  };
};
}

#endif // RELAXEDBARRIERPENALTY_OCS2_H
