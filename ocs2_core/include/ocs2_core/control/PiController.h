#pragma once

#include "ocs2_core/control/Controller.h"

#include <random>

#include <ocs2_core/cost/PathIntegralCostFunction.h>
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"

namespace ocs2 {

/**
 * The base class for all controllers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
// TODO(jcarius) do we need the LOGIC_RULES_T template?

template <size_t STATE_DIM, size_t INPUT_DIM>
class PiController : public Controller<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Controller<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename Base::scalar_t;
  using dimensions_t = typename Base::dimensions_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_matrix_t = typename dimensions_t::input_matrix_t;

  using logic_rules_t = NullLogicRules;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using cost_function_t = PathIntegralCostFunction<STATE_DIM, INPUT_DIM, logic_rules_t>;

  /**
   * Constructor.
   */
  PiController(const constraint_t& constraints, const cost_function_t& costs, const scalar_t rollout_dt, const scalar_t noiseScaling)
      : constraints_(constraints),
        costs_(costs),
        rollout_dt_(rollout_dt),
        standardNormalDistribution_(scalar_t(0.0), scalar_t(1.0)),
        gamma_(noiseScaling) {
    eigenRandomNormalNullaryExpr_ = [&](scalar_t) -> scalar_t {
      return standardNormalDistribution_(generator_);
    };  // dummy argument required by Eigen
  }

  /**
   * Default destructor.
   */
  virtual ~PiController() = default;

  /**
   * Computes noisy control command that keeps the system within the constraints
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current input.
   */
  virtual input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    // extract constraint terms
    constraints_.setCurrentStateAndControl(t, x, input_vector_t::Zero());
    typename constraint_t::constraint1_vector_t c;
    constraints_.getConstraint1(c);
    typename constraint_t::constraint1_input_matrix_t D;
    constraints_.getConstraint1DerivativesControl(D);

    // extract cost terms
    costs_.setCurrentStateAndControl(t, x, input_vector_t::Zero());
    V_ = costs_.getIntermediateCost();                    // must have set zero input before
    costs_.getIntermediateCostSecondDerivativeInput(R_);  // TODO(jcarius) do we need a R *= 2; here?
    Rinv_ = R_.inverse();                                 // TODO(jcarius) better way to to this?
    costs_.getIntermediateCostDerivativeInput(r_);        // must have set zero input before

    // calculate auxiliary terms
    Ddagger_ = Rinv_ * D.transpose() * (D * Rinv_ * D.transpose()).inverse();
    Dtilde_ = Ddagger_ * D;

    input_matrix_t QQt = gamma_ * (input_matrix_t::Identity() - Dtilde_) * Rinv_;
    input_matrix_t Q = QQt.llt().matrixL();

    input_vector_t constraintInput = (input_matrix_t::Identity() - Dtilde_) * Rinv_ * r_ - Ddagger_ * c_;
    noiseInput_ = Q / std::sqrt(rollout_dt_) * input_vector_t::NullaryExpr(eigenRandomNormalNullaryExpr_);

    return constraintInput + noiseInput_;
  }

 public:
  // values set in computeInput method
  scalar_t V_;
  input_matrix_t R_;
  input_matrix_t Rinv_;
  input_vector_t r_;
  typename constraint_t::constraint1_vector_t c_;
  Eigen::Matrix<scalar_t, -1, -1> Ddagger_;
  input_matrix_t Dtilde_;
  input_vector_t noiseInput_;

 protected:
  constraint_t constraints_;
  cost_function_t costs_;
  scalar_t rollout_dt_;
  scalar_t gamma_;  //! scaling of noise

  // random number generator
  // TODO(jcarius) thready safety of these objects??
  std::default_random_engine generator_;
  std::normal_distribution<scalar_t> standardNormalDistribution_;
  std::function<scalar_t(scalar_t)> eigenRandomNormalNullaryExpr_;
};

}  // namespace ocs2
