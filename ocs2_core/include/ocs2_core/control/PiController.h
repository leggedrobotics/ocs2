#pragma once

#include "ocs2_core/control/ControllerBase.h"

#include <random>

#include <ocs2_core/cost/PathIntegralCostFunction.h>
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
template <size_t STATE_DIM, size_t INPUT_DIM, typename LOGIC_RULES_T = NullLogicRules>
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

  using logic_rules_t = LOGIC_RULES_T;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using cost_function_t = PathIntegralCostFunction<STATE_DIM, INPUT_DIM, logic_rules_t>;

  struct PiControllerEvaluationData {
    scalar_t t_;        //! time of evaluation
    state_vector_t x_;  //! state
    input_vector_t u_;  //! calculated input
    scalar_t V_;
    input_matrix_t R_;
    input_matrix_t Rinv_;
    input_vector_t r_;
    dynamic_vector_t c_;
    dynamic_matrix_t Ddagger_;
    input_matrix_t Dtilde_;
    input_vector_t noiseInput_;
  };

  /**
   * Constructor with full options
   * @param[in] constraints Pointer to constraint function. Assumes persistence as long as controller survives
   * @param[in] consts Pointer to cost function. Assumes persistence as long as controller survives
   * @param[in] rollout_dt Time step to use in rollout
   * @param[in] noiseScaling The level of noise
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

    linInterpolateXNominal_.setZero();
    linInterpolateUff_.setZero();

    // check constraints
    if (constraints_->numStateOnlyConstraint(0.0)) {
      throw std::runtime_error("PiController does not support state-only constraints");
    }
    if (constraints_->numInequalityConstraint(0.0)) {
      throw std::runtime_error("PiController does not support inequality constraints");
    }
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
    // extract cost terms
    scalar_t V;
    costs_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
    costs_->getIntermediateCost(V);  // must have set zero input before
    input_matrix_t R, Rinv;
    costs_->getIntermediateCostSecondDerivativeInput(R);  // TODO(jcarius) do we need a R *= 2; here?
    Rinv = R.ldlt().solve(input_matrix_t::Identity());
    input_vector_t r;
    costs_->getIntermediateCostDerivativeInput(r);  // must have set zero input before

    // extract constraint terms and calculate auxiliary quantities
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

    input_matrix_t QQt = gamma_ * (input_matrix_t::Identity() - Dtilde) * Rinv;
    input_matrix_t Q;
    if (!QQt.isZero()) {
      Q = QQt.llt().matrixL();
    } else {
      Q.setZero();
    }

    const input_vector_t constrainedInput = -(input_matrix_t::Identity() - Dtilde) * Rinv * r - Ddagger * c;
    const input_vector_t noiseInput = Q / std::sqrt(rollout_dt_) * input_vector_t::NullaryExpr(eigenRandomNormalNullaryExpr_);

    input_vector_t uff;
    linInterpolateUff_.interpolate(t, uff);

    const input_vector_t result = constrainedInput + noiseInput + uff;

    if (cacheResults_) {
      cacheData_.emplace_back(PiControllerEvaluationData());
      auto& data = cacheData_.back();
      data.t_ = t;
      data.x_ = x;
      data.u_ = result;
      data.V_ = V;
      data.R_ = R;
      data.Rinv_ = Rinv;
      data.r_ = r;
      data.c_ = c;
      data.Ddagger_ = Ddagger;
      data.Dtilde_ = Dtilde;
      data.noiseInput_ = noiseInput;
    }

    return result;
  }

  virtual void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    throw std::runtime_error("not implemented");

    //      flatArray.clear();
    //      flatArray.reserve(time_.size() * (1+STATE_DIM+INPUT_DIM));

    //      std::copy(time_.begin(), time_.end(), std::back_inserter(flatArray));

    //      for(const auto& x : xNominal_){
    //          for(size_t i=0; i<STATE_DIM; i++){
    //                flatArray.push_back(x(i));
    //          }
    //      }

    //      for(const auto& u: uff_){
    //          for(size_t i=0; i<INPUT_DIM; i++){
    //              flatArray.push_back(u(i));
    //          }
    //      }
  }

  virtual void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    throw std::runtime_error("not implemented");

    //      const auto size = flatArray.size() / (1+STATE_DIM+INPUT_DIM);

    //      time_.clear();
    //      time_.reserve(size);
    //      time_.assign(flatArray.begin(), flatArray.begin()+size);

    //      xNominal_.clear();
    //      xNominal_.reserve(size);
    //      for(size_t i=0; i<size; i++){
    //          xNominal_.emplace_back(state_vector_t(flatArray.data() + size + i * STATE_DIM));
    //      }

    //      uff_.clear();
    //      uff_.reserve(size);
    //      for(size_t i=0; i<size; i++){
    //          uff_.emplace_back(input_vector_t(flatArray.data() + size + size * STATE_DIM + i * INPUT_DIM));
    //      }

    //      linInterpolateXNominal_.setTimeStamp(&time_);
    //      linInterpolateXNominal_.setData(&xNominal_);
    //      linInterpolateUff_.setTimeStamp(&time_);
    //      linInterpolateUff_.setData(&uff_);
  }

  /**
   * @brief setFeedforwardInputAndState: Assign
   * @param[in] time Time trajectory
   * @param[in] xNominal Nominal state trajectory
   * @param[in] uff Feedforward input trajectory
   */
  void setFeedforwardInputAndState(const scalar_array_t& time, const state_vector_array_t& xNominal, const input_vector_array_t& uff) {
    if ((time.size() != xNominal.size()) or (time.size() != uff.size())) {
      throw std::runtime_error("PiController::setFeedforwardInputAndState -- Sizes don't match.");
    }
    time_ = time;
    xNominal_ = xNominal;
    uff_ = uff;

    linInterpolateXNominal_.setData(&time_, &xNominal_);
    linInterpolateUff_.setData(&time_, &uff_);
  }

  virtual void swap(PiController<STATE_DIM, INPUT_DIM>& other) { throw std::runtime_error("not implemented"); }

  virtual ControllerType getType() const override { return ControllerType::PATH_INTEGRAL; }

  virtual void clear() override {
    time_.clear();
    xNominal_.clear();
    uff_.clear();
  }

  virtual void setZero() override {
    linInterpolateXNominal_.setZero();
    linInterpolateUff_.setZero();
    gamma_ = 0.0;
  }

  virtual bool empty() const override { return time_.empty(); }

  void setRandomSeed(unsigned int seed) { generator_.seed(seed); }

 public:
  scalar_t gamma_;     //! scaling of noise
  bool cacheResults_;  //! whether or not to keep recording results

  std::vector<PiControllerEvaluationData> cacheData_;

 protected:
  constraint_t* constraints_;  //! pointer to constraint from solver
  cost_function_t* costs_;     //! pointer to cost function from solver
  scalar_t rollout_dt_;

  scalar_array_t time_;                                              //! time array for uff and xNominal
  state_vector_array_t xNominal_;                                    //! nominal state trajectory w/out noise
  input_vector_array_t uff_;                                         //! feedforward inputs
  EigenLinearInterpolation<state_vector_t> linInterpolateXNominal_;  //! interpolation of xNominal_
  EigenLinearInterpolation<input_vector_t> linInterpolateUff_;       //! interpolation of uff_

  // random number generator
  // TODO(jcarius) thread safety of these objects??
  std::default_random_engine generator_;
  std::normal_distribution<scalar_t> standardNormalDistribution_;
  std::function<scalar_t(scalar_t)> eigenRandomNormalNullaryExpr_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(PiController<STATE_DIM, INPUT_DIM>& a, PiController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
