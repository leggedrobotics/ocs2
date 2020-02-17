

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
template <size_t STATE_DIM, size_t INPUT_DIM>
class PenaltyBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;
  using dynamic_matrix_array_t = typename DIMENSIONS::dynamic_matrix_array_t;

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
   * @param [in] h: Vector of inequality constraint values
   * @param [out] penalty: The penalty cost.
   */
  void getPenaltyCost(const scalar_array_t& h, scalar_t& penalty) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [out] penaltyDerivativeState: Derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                     state_vector_t& penaltyDerivativeState) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input vector.
   * @param [out] penaltyDerivativeInput: Derivative of the penalty cost with respect to input vector.
   */
  void getPenaltyCostDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu,
                                     input_vector_t& penaltyDerivativeInput) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] ddhdxdx: Vector of inequality constraint second derivatives with respect to state.
   * @param [out] penaltySecondDerivativeState: Second derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                           const dynamic_matrix_array_t& ddhdxdx, state_matrix_t& penaltySecondDerivativeState) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudu: Vector of inequality constraint second derivatives with respect to input.
   * @param [out] penaltySecondDerivativeInput: Second derivative of the penalty cost with respect to input.
   */
  void getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu,
                                           const dynamic_matrix_array_t& ddhdudu, input_matrix_t& penaltySecondDerivativeInput) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudx: Vector of inequality constraint derivatives with respect to input and state.
   * @param [out] penaltyDerivativeInputState: Derivative of the penalty cost with respect to input and state.
   */
  void getPenaltyCostDerivativeInputState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx, const dynamic_vector_array_t& dhdu,
                                          const dynamic_matrix_array_t& ddhdudx, input_state_matrix_t& penaltyDerivativeInputState) const;

  /**
   * Computes the sum of squared constraint violation.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [out] squaredViolation: sum of squared constraint violation.
   */
  void getConstraintViolationSquaredNorm(const scalar_array_t& h, scalar_t& squaredViolation) const;

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
  virtual scalar_t getPenaltyFunctionSecondDerivative(scalar_t h) const = 0;
};
}  // namespace ocs2

#include "implementation/PenaltyBase.h"

#endif  // PENALTYBASE_OCS2_H
