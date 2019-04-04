//
// Created by rgrandia on 06.02.19.
//

#ifndef PENALTYBASE_OCS2_H
#define PENALTYBASE_OCS2_H

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
 *   Implements the cost penalty for the inequality constraint
 *   \f$ h_i(x, u) \geq 0 \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty = \f$ \sum_{i=1}^{M} p(h_i(x, u)) \f$
 *
 *   The scalar penalty function \f$ p() \f$ and its derivatives are to be implemented in the derived class.
 *   This base class implements the chain rule from the inequality constraint and the implemented penalty function.
 */
template<size_t STATE_DIM, size_t INPUT_DIM>
class PenaltyBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
  typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;
  typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;

  /**
  * Default constructor
  */
  PenaltyBase() = default;

  /**
  * Default destructor
  */
  virtual ~PenaltyBase() = default;

  /**
   * Get the penalty cost.
   *
   * @param [in] numInequalityConstraints
   * @param [in] h: Vector of inequality constraint values
   * @return
   */
  scalar_t getPenaltyCost(size_t numInequalityConstraints, const scalar_array_t &h) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] numInequalityConstraints
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [out] penaltyDerivativeState: Derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostDerivativeState(size_t numInequalityConstraints,
                                     const scalar_array_t &h, const state_vector_array_t &dhdx,
                                     state_vector_t &penaltyDerivativeState) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] numInequalityConstraints
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input vector.
   * @param [out] penaltyDerivativeInput: Derivative of the penalty cost with respect to input vector.
   */
  void getPenaltyCostDerivativeInput(size_t numInequalityConstraints, const scalar_array_t &h,
                                     const input_vector_array_t &dhdu,
                                     input_vector_t &penaltyDerivativeInput) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] numInequalityConstraints
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] ddhdxdx: Vector of inequality constraint second derivatives with respect to state.
   * @param [out] penaltySecondDerivativeState: Second derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostSecondDerivativeState(size_t numInequalityConstraints, const scalar_array_t &h,
                                           const state_vector_array_t &dhdx,
                                           const state_matrix_array_t &ddhdxdx,
                                           state_matrix_t &penaltySecondDerivativeState) const;

  /**
  * Second derivative of penalty cost.
  *
  * @param [in] numInequalityConstraints
  * @param [in] h: Vector of inequality constraint values.
  * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
  * @param [in] ddhdudu: Vector of inequality constraint second derivatives with respect to input.
  * @param [out] penaltySecondDerivativeInput: Second derivative of the penalty cost with respect to input.
  */
  void getPenaltyCostSecondDerivativeInput(size_t numInequalityConstraints, const scalar_array_t &h,
                                           const input_vector_array_t &dhdu,
                                           const input_matrix_array_t &ddhdudu,
                                           input_matrix_t &penaltySecondDerivativeInput) const;

  /**
  * Second derivative of penalty cost.
  *
  * @param [in] numInequalityConstraints
  * @param [in] h: Vector of inequality constraint values.
  * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
  * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
  * @param [in] ddhdudx: Vector of inequality constraint derivatives with respect to input and state.
  * @param [out] penaltyDerivativeInputState: Derivative of the penalty cost with respect to input and state.
  */
  void getPenaltyCostDerivativeInputState(size_t numInequalityConstraints, const scalar_array_t &h,
                                          const state_vector_array_t &dhdx,
                                          const input_vector_array_t &dhdu,
                                          const input_state_matrix_array_t &ddhdudx,
                                          input_state_matrix_t &penaltyDerivativeInputState) const;

  /**
   * Computes the sum of squared constraint violation.
   *
   * @param [in] numInequalityConstraints
   * @param [in] h: Vector of inequality constraint values.
   * @return sum of squared constraint violation.
   */
  scalar_t getConstraintViolationSquaredNorm(size_t numInequalityConstraints, const scalar_array_t &h) const;

 private:

  /**
   * Compute the penalty value at a certain constraint value.
   *
   * @param [in] h: Constraint value.
   * @return penalty cost.
   */
  virtual scalar_t getPenaltyFunctionValue(scalar_t h) const = 0;

  /**
  * Compute the penalty derivative at a certain constraint value.
  *
  * @param [in] h: Constraint value.
  * @return penalty derivative with respect to constraint value.
  */
  virtual scalar_t getPenaltyFunctionDerivative(scalar_t h) const = 0;

  /**
  * Compute the penalty second derivative at a certain constraint value.
  *
  * @param [in] h: Constraint value.
  * @return penalty second derivative with respect to constraint value.
  */
  virtual scalar_t getPenaltyFunctionHessian(scalar_t h) const = 0;
};
}

#include "implementation/PenaltyBase.h"

#endif //PENALTYBASE_OCS2_H
