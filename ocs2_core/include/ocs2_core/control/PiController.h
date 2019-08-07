#pragma once

#include "ocs2_core/control/ControllerBase.h"

#include <random>

#include <ocs2_core/cost/CostFunctionBase.h>
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * The base class for all controllers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class PiController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename Base::scalar_t;
  using scalar_array_t = typename Base::scalar_array_t;
  using float_array_t = typename Base::float_array_t;
  using dimensions_t = typename Base::dimensions_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using state_vector_array_t = typename dimensions_t::state_vector_array_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_matrix_t = typename dimensions_t::input_matrix_t;
  using dynamic_vector_t = typename Base::dimensions_t::dynamic_vector_t;
  using dynamic_matrix_t = typename Base::dimensions_t::dynamic_matrix_t;
  using constraint1_vector_t = typename Base::dimensions_t::constraint1_vector_t;
  using constraint1_input_matrix_t = typename Base::dimensions_t::constraint1_input_matrix_t;
  using input_constraint1_matrix_t = typename Base::dimensions_t::input_constraint1_matrix_t;

  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using cost_function_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;

  /**
   * @brief PiControllerEvaluationData struct is used to cache data.
   * It will be extracted from the PI solver to avoid re-computing some quantities.
   */
  struct PiControllerEvaluationData {
    scalar_t t_;          //! time of evaluation
    state_vector_t x_;    //! state
    input_vector_t u_;    //! calculated input (including noise)
    scalar_t stageCost_;  //! running cost (intermediate) incurred
  };

  /**
   * Constructor with full options
   * @param[in] constraints Pointer to constraint function. Assumes persistence as long as controller survives
   * @param[in] costs Pointer to cost function. Assumes persistence as long as controller survives
   * @param[in] rollout_dt Time step to use in rollout
   * @param[in] noiseScaling The level of noise (temperature)
   */
  PiController(constraint_t* constraints, cost_function_t* costs, const scalar_t rollout_dt, const scalar_t noiseScaling)
      : constraints_(constraints),
        costs_(costs),
        rollout_dt_(rollout_dt),
        standardNormalDistribution_(scalar_t(0.0), scalar_t(1.0)),
        gamma_(noiseScaling),
        cacheResults_(false) {
    eigenRandomNormalNullaryExpr_ = [this](scalar_t) -> scalar_t {
      return standardNormalDistribution_(generator_);
    };  // dummy argument required by Eigen

    // check constraints
    if (constraints_->numStateOnlyConstraint(0.0)) {
      throw std::runtime_error("PiController does not support state-only constraints");
    }
    if (constraints_->numInequalityConstraint(0.0)) {
      throw std::runtime_error("PiController does not support inequality constraints");
    }
  }

  /**
   * Copy constructor
   */
  PiController(const PiController& other) : PiController(other.constraints_, other.costs_, other.rollout_dt_, other.gamma_) {
    samplingPolicy_.reset(other.samplingPolicy_->clone());
  }

  PiController<STATE_DIM, INPUT_DIM>* clone() override { return new PiController<STATE_DIM, INPUT_DIM>(*this); }

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
    // extract cost terms
    costs_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
    input_matrix_t R, Rinv;
    costs_->getIntermediateCostSecondDerivativeInput(R);
    Rinv = R.ldlt().solve(input_matrix_t::Identity());
    input_vector_t r;
    costs_->getIntermediateCostDerivativeInput(r);  // must have set zero input before

    // extract constraint terms and calculate auxiliary quantities
    // TODO(jcarius) Use Ruben's functions to do these projections
    dynamic_vector_t c;
    dynamic_matrix_t Ddagger;
    input_matrix_t Dtilde;
    const auto nc = constraints_->numStateInputConstraint(t);
    if (nc) {
      constraints_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
      constraint1_vector_t c_full;
      constraints_->getConstraint1(c_full);
      c = c_full.topRows(nc);
      constraint1_input_matrix_t D_full;
      constraints_->getConstraint1DerivativesControl(D_full);
      dynamic_matrix_t D = D_full.topRows(nc);

      Ddagger = Rinv * D.transpose() * (D * Rinv * D.transpose()).ldlt().solve(dynamic_matrix_t::Identity(nc, nc));
      Dtilde = Ddagger * D;
    } else {
      c = constraint1_vector_t::Zero();
      Ddagger = input_constraint1_matrix_t::Zero();
      Dtilde.setZero();
    }

    // noise covariance
    const input_matrix_t QQt = gamma_ * (input_matrix_t::Identity() - Dtilde) * Rinv;
    input_matrix_t Q;
    if (!QQt.isZero()) {
      Q = QQt.llt().matrixL();
    } else {
      Q.setZero();
    }

    // compute input
    const input_vector_t defaultInput = -(input_matrix_t::Identity() - Dtilde) * Rinv * r - Ddagger * c;

    input_vector_t policyInput;
    if (samplingPolicy_) {
      policyInput = (input_matrix_t::Identity() - Dtilde) * samplingPolicy_->computeInput(t, x) - Ddagger * c;
    } else {
      policyInput = defaultInput;
    }

    const input_vector_t noiseInput = Q / std::sqrt(rollout_dt_) * input_vector_t::NullaryExpr(eigenRandomNormalNullaryExpr_);
    const input_vector_t totalInput = policyInput + noiseInput;

    if (cacheResults_) {
      // compute running cost incurred
      scalar_t V;
      costs_->getIntermediateCost(V);  // must have set zero input before
      scalar_t stageCost = 0.5 * policyInput.dot(R * policyInput) + r.dot(policyInput) + V;
      if (samplingPolicy_) {
        stageCost += (policyInput - defaultInput).dot(R * noiseInput);
      }

      cacheData_.emplace_back(PiControllerEvaluationData());
      auto& data = cacheData_.back();
      data.t_ = t;
      data.x_ = x;
      data.u_ = totalInput;
      data.stageCost_ = stageCost;
    }

    return totalInput;
  }

  virtual void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    throw std::runtime_error("not implemented");
  }

  virtual void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    throw std::runtime_error("not implemented");
  }

  void concatenate(const Base* nextController) override { throw std::runtime_error("not implemented"); }

  /**
   * @brief setSamplingPolicy Allows setting a controller for importance sampling (warm-starting)
   * @param samplingPolicy Unique pointer to the policy
   */
  void setSamplingPolicy(std::unique_ptr<ControllerBase<STATE_DIM, INPUT_DIM>> samplingPolicy) {
    samplingPolicy_ = std::move(samplingPolicy);
  }

  virtual void swap(PiController<STATE_DIM, INPUT_DIM>& other) { throw std::runtime_error("not implemented"); }

  virtual ControllerType getType() const override { return ControllerType::PATH_INTEGRAL; }

  virtual void clear() override { samplingPolicy_.reset(); }

  virtual void setZero() override {
    samplingPolicy_.reset();
    gamma_ = 0.0;
  }

  virtual bool empty() const override { return samplingPolicy_ == nullptr; }

  void setRandomSeed(unsigned int seed) { generator_.seed(seed); }

  void display() const override {
    if (samplingPolicy_) {
      std::cerr << "Sampling policy:" << std::endl;
      samplingPolicy_->display();
    } else {
      std::cerr << "Sampling policy empty." << std::endl;
    }
  }

 public:
  scalar_t gamma_;     //! scaling of noise
  bool cacheResults_;  //! whether or not to keep recording results

  std::vector<PiControllerEvaluationData> cacheData_;

 protected:
  constraint_t* constraints_;  //! pointer to constraint from solver
  cost_function_t* costs_;     //! pointer to cost function from solver
  scalar_t rollout_dt_;        //! time step during rollout

  std::unique_ptr<ControllerBase<STATE_DIM, INPUT_DIM>> samplingPolicy_;  //! the underlying policy to be used

  // TODO(jcarius) thread safety of these objects??
  std::default_random_engine generator_;  //! random number generator
  std::normal_distribution<scalar_t> standardNormalDistribution_;
  std::function<scalar_t(scalar_t)> eigenRandomNormalNullaryExpr_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(PiController<STATE_DIM, INPUT_DIM>& a, PiController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
