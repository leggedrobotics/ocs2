#pragma once

#include <ocs2_double_slit_example/definitions.h>
#include "ocs2_core/cost/CostFunctionBase.h"

#include <functional>

namespace ocs2 {
namespace double_slit {

/**
 * Cost Function for the double slit
 */
class DoubleSlitBarrierCost final : public CostFunctionBase<DoubleSlit::STATE_DIM_, DoubleSlit::STATE_DIM_, NullLogicRules> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBase<DoubleSlit::STATE_DIM_, DoubleSlit::STATE_DIM_, NullLogicRules> ;

  using DIMENSIONS = Dimensions<DoubleSlit::STATE_DIM_, DoubleSlit::STATE_DIM_> ;
  using scalar_t = typename DIMENSIONS::scalar_t ;
  using state_vector_t = typename DIMENSIONS::state_vector_t ;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t ;
  using input_vector_t = typename DIMENSIONS::input_vector_t ;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t ;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t ;

  using potential_fct_t = std::function<scalar_t(const state_vector_t&, scalar_t)> ;
  using final_cost_fct_t =  std::function<scalar_t(const state_vector_t&)> ;
  using r_fct_t = std::function<input_vector_t(const state_vector_t&, scalar_t)> ;

  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(u-u_{n})' R (u-u_{n}) + V(x) + r'(x) u \f$
   * - \f$ \Phi(x) \f$ arbitrary
   * @param [in] R: \f$ R \f$
   * @param [in] uNominalIntermediate: \f$ u_{n}\f$
   * @param [in] V: \f$ V(x,t) \f$
   * @param [in] r: \f$ r(x,t) \f$
   */
  DoubleSlitBarrierCost(input_matrix_t R, input_vector_t uNominalIntermediate, potential_fct_t V, r_fct_t r,
                        final_cost_fct_t Phi)
      : rM_(std::move(R)), uNominalIntermediate_(std::move(uNominalIntermediate)), v_(std::move(V)), r_(std::move(r)), phi_(std::move(Phi)) {}

  /**
   * Destructor
   */
  ~DoubleSlitBarrierCost() override = default;

  DoubleSlitBarrierCost* clone() const override { return new DoubleSlitBarrierCost(*this); }

  void getIntermediateCost(scalar_t& L) override {
    L = scalar_t(0.5) * (this->u_ - uNominalIntermediate_).dot(rM_ * (this->u_ - uNominalIntermediate_)) + v_(this->x_, this->t_) +
        r_(this->x_, this->t_).dot(this->u_);
  }

  void getIntermediateCostDerivativeState(state_vector_t&) override {
    throw std::runtime_error("getIntermediateCostDerivativeState not implemented.");
  }

  void getIntermediateCostSecondDerivativeState(state_matrix_t&) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override {
    dLdu = rM_ * (this->u_ - uNominalIntermediate_) + r_(this->x_, this->t_);
  }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override { dLduu = rM_; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t&) override {
    throw std::runtime_error("getIntermediateCostSecondDerivativeState not implemented.");
  }

  void getTerminalCost(scalar_t& Phi) override { Phi = phi_(this->x_); }

  void getTerminalCostDerivativeState(state_vector_t&) override {
    throw std::runtime_error("getTerminalCostDerivativeState not implemented.");
  }

  void getTerminalCostSecondDerivativeState(state_matrix_t&) override {
    throw std::runtime_error("getTerminalCostSecondDerivativeState not implemented.");
  }

 protected:
  input_matrix_t rM_;

  input_vector_t uNominalIntermediate_;

  potential_fct_t v_;
  r_fct_t r_;
  final_cost_fct_t phi_;
};

}  // namespace double_slit
}  // namespace ocs2
