#pragma once

#include <ocs2_double_slit_example/definitions.h>
#include "ocs2_core/cost/CostFunctionBase.h"

#include <functional>

namespace ocs2 {
namespace double_slit {

/**
 * Cost Function for the double slit
 */
class DoubleSlitBarrierCost final : public CostFunctionBase<double_slit::STATE_DIM_, double_slit::STATE_DIM_, NullLogicRules> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef CostFunctionBase<double_slit::STATE_DIM_, double_slit::STATE_DIM_, NullLogicRules> BASE;

  typedef Dimensions<double_slit::STATE_DIM_, double_slit::STATE_DIM_> DIMENSIONS;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;

  typedef std::function<scalar_t(const state_vector_t&, scalar_t)> potential_fct_t;
  typedef std::function<scalar_t(const state_vector_t&)> final_cost_fct_t;
  typedef std::function<input_vector_t(const state_vector_t&, scalar_t)> r_fct_t;

  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(u-u_{n})' R (u-u_{n}) + V(x) + r'(x) u \f$
   * - \f$ \Phi(x) \f$ arbitrary
   * @param [in] R: \f$ R \f$
   * @param [in] uNominalIntermediate: \f$ u_{n}\f$
   * @param [in] V: \f$ V(x,t) \f$
   * @param [in] r: \f$ r(x,t) \f$
   */
  DoubleSlitBarrierCost(const input_matrix_t& R, const input_vector_t& uNominalIntermediate, potential_fct_t V, r_fct_t r,
                        final_cost_fct_t Phi)
      : R_(R), uNominalIntermediate_(uNominalIntermediate), V_(std::move(V)), r_(std::move(r)), Phi_(std::move(Phi)) {}

  /**
   * Destructor
   */
  virtual ~DoubleSlitBarrierCost() = default;

  virtual DoubleSlitBarrierCost* clone() const override { return new DoubleSlitBarrierCost(*this); }

  virtual void getIntermediateCost(scalar_t& L) override {
    L = scalar_t(0.5) * (this->u_ - uNominalIntermediate_).dot(R_ * (this->u_ - uNominalIntermediate_)) + V_(this->x_, this->t_) +
        r_(this->x_, this->t_).dot(this->u_);
  }

  virtual void getIntermediateCostDerivativeState(state_vector_t&) override {
    throw std::runtime_error("getIntermediateCostDerivativeState not implemented.");
  }

  virtual void getIntermediateCostSecondDerivativeState(state_matrix_t&) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override {
    dLdu = R_ * (this->u_ - uNominalIntermediate_) + r_(this->x_, this->t_);
  }

  virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override { dLduu = R_; }

  virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t&) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  virtual void getTerminalCost(scalar_t& Phi) override { Phi = Phi_(this->x_); }

  virtual void getTerminalCostDerivativeState(state_vector_t&) override {
    throw std::runtime_error("getTerminalCostDerivativeState not implemented.");
  }

  virtual void getTerminalCostSecondDerivativeState(state_matrix_t&) override {
    throw std::runtime_error("getTerminalCostSecondDerivativeState not implemented.");
  }

 protected:
  input_matrix_t R_;
  input_vector_t uNominalIntermediate_;

  potential_fct_t V_;
  r_fct_t r_;
  final_cost_fct_t Phi_;
};

}  // namespace double_slit
}  // namespace ocs2
