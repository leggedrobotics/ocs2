#pragma once

#include "ocs2_core/cost/CostFunctionBase.h"

#include <functional>

namespace ocs2 {

/**
 * Cost Function for Path Integral Solver
 * Requires quadratic costs on the input and arbitrary state cost.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T = NullLogicRules>
class PathIntegralCostFunction final : public CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;

  typedef std::function<scalar_t(const state_vector_t&)> potential_fct_t;
  typedef std::function<input_vector_t(const state_vector_t&)> r_fct_t;

  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(u-u_{n})' R (u-u_{n}) + V(x) + r'(x) u \f$
   * - \f$ \Phi(x) \f$ arbitrary
   * @param [in] R: \f$ R \f$
   * @param [in] uNominalIntermediate: \f$ u_{n}\f$
   * @param [in] V: \f$ V(x) \f$
   * @param [in] r: \f$ r'(x) \f$
   */
  PathIntegralCostFunction(const input_matrix_t& R, const input_vector_t& uNominalIntermediate, potential_fct_t V, r_fct_t r,
                           potential_fct_t Phi)
      : R_(R), uNominalIntermediate_(uNominalIntermediate), V_(std::move(V)), r_(std::move(r)), Phi_(std::move(Phi)) {}

  /**
   * Destructor
   */
  virtual ~PathIntegralCostFunction() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual PathIntegralCostFunction* clone() const override {
    return new PathIntegralCostFunction<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
  }

  /**
   * Get the intermediate cost.
   *
   * @param [out] L: The intermediate cost value.
   */
  virtual void getIntermediateCost(scalar_t& L) override {
    L = scalar_t(0.5) * (this->u_ - uNominalIntermediate_).dot(R_ * (this->u_ - uNominalIntermediate_)) + V_(this->x_) + r_(this->x_).dot(this->u_);
  }

  /**
   * Get the state derivative of the intermediate cost.
   *
   * @param [out] dLdx: First order derivative of the intermediate cost with respect to state vector.
   */
  virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) override {
    throw std::runtime_error("getIntermediateCostDerivativeState not implemented.");
  }

  /**
   * Get state second order derivative of the intermediate cost.
   *
   * @param [out] dLdxx: Second order derivative of the intermediate cost with respect to state vector.
   */
  virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  /**
   * Get control input derivative of the intermediate cost.
   *
   * @param [out] dLdu: First order derivative of the intermediate cost with respect to input vector.
   */
  virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override {
    dLdu = R_ * (this->u_ - uNominalIntermediate_) + r_(this->x_);
  }

  /**
   * Get control input second derivative of the intermediate cost.
   *
   * @param [out] dLduu: Second order derivative of the intermediate cost with respect to input vector.
   */
  virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override { dLduu = R_; }

  /**
   * Get the input-state derivative of the intermediate cost.
   *
   * @param [out] dLdux: Second order derivative of the intermediate cost with respect to input vector and state.
   */
  virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  /**
   * Get the terminal cost.
   *
   * @param [out] Phi: The final cost value.
   */
  virtual void getTerminalCost(scalar_t& Phi) override { Phi = Phi_(this->x_); }

  /**
   * Get the terminal cost state derivative of the terminal cost.
   *
   * @param [out] dPhidx: First order final cost derivative with respect to state vector.
   */
  virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) override {
    throw std::runtime_error("getTerminalCostDerivativeState not implemented.");
  }

  /**
   * Get the terminal cost state second derivative of the terminal cost.
   *
   * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
   */
  virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override {
    throw std::runtime_error("getTerminalCostSecondDerivativeState not implemented.");
  }

 protected:
  input_matrix_t R_;
  input_vector_t uNominalIntermediate_;

  potential_fct_t V_;
  r_fct_t r_;
  potential_fct_t Phi_;
};
}  // namespace ocs2
