/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_BASE.h>
#include <ocs2_slq/SLQ_DataCollector.h>
#include <ocs2_slq/SLQ_MP.h>

#include <ocs2_frank_wolfe/NLP_Cost.h>

#include <ocs2_ocs2/GDDP.h>

namespace ocs2 {

/**
 * This class is an interface to a NLP cost for the upper level optimization of OCS2.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class UpperLevelCost final : public NLP_Cost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef NLP_Cost BASE;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_matrix_t = typename BASE::dynamic_matrix_t;

  typedef GDDP<STATE_DIM, INPUT_DIM> gddp_t;
  typedef SLQ_BASE<STATE_DIM, INPUT_DIM> slq_t;
  typedef SLQ<STATE_DIM, INPUT_DIM> slq_st_t;
  typedef SLQ_MP<STATE_DIM, INPUT_DIM> slq_mt_t;
  typedef SLQ_DataCollector<STATE_DIM, INPUT_DIM> slq_data_collector_t;

  using state_vector_t = typename slq_t::state_vector_t;
  using controlled_system_base_t = typename slq_t::controlled_system_base_t;
  using derivatives_base_t = typename slq_t::derivatives_base_t;
  using constraint_base_t = typename slq_t::constraint_base_t;
  using cost_function_base_t = typename slq_t::cost_function_base_t;
  using operating_trajectories_base_t = typename slq_t::operating_trajectories_base_t;

  /**
   * Constructor
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  UpperLevelCost(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                 const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings = SLQ_Settings(),
                 std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr,
                 bool display = false, const GDDP_Settings& gddpSettings = GDDP_Settings())
      : slqPtr_(new slq_st_t(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
                             settings, logicRulesPtr, heuristicsFunctionPtr)),
        slqDataCollectorPtr_(new slq_data_collector_t(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr)),
        gddpPtr_(new gddp_t(gddpSettings)),
        display_(display) {}

  /**
   * Default destructor.
   */
  ~UpperLevelCost() = default;

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   */
  void setDDP(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
              const scalar_array_t& partitioningTimes) {
    initTime_ = initTime;
    initState_ = initState;
    finalTime_ = finalTime;
    partitioningTimes_ = partitioningTimes;
  }

  size_t setCurrentParameter(const dynamic_vector_t& x) override {
    // set event time
    eventTimes_ = scalar_array_t(x.data(), x.data() + x.size());
    slqPtr_->getLogicRulesPtr()->eventTimes() = eventTimes_;
    slqPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();

    // run SLQ
    try {
      slqPtr_->reset();
      slqPtr_->run(initTime_, initState_, finalTime_, partitioningTimes_);
    } catch (const std::exception& e) {
      std::cerr << "\t     exception: " << e.what();
      return 0;
    }

    // get the cost and constraints ISE
    slqPtr_->getPerformanceIndeces(cost_, constraintISE1_, constraintISE2_);

    // display
    if (display_) {
      std::cerr << "\t     constraintISE1: " << constraintISE1_ << "\t     constraintISE2: " << constraintISE2_
                << "\t#Iterations: " << slqPtr_->getNumIterations() << std::endl;
    }

    // collect the SLQ solution
    slqDataCollectorPtr_->collect(slqPtr_.get());

    // status is false if the constraints ISE is higher than minAbsConstraint1RMSE_
    bool status1 = (constraintISE1_ <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;
    bool status2 = (constraintISE2_ <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;

    status_ = status1 && status2;
    return 0;
  }

  bool getCost(size_t id, scalar_t& f) override {
    f = cost_;
    return status_;
  }

  void getCostDerivative(size_t id, dynamic_vector_t& g) override {
    gddpPtr_->run(eventTimes_, slqDataCollectorPtr_.get());
    gddpPtr_->getCostFuntionDerivative(g);
  }

  void getCostSecondDerivative(size_t id, dynamic_matrix_t& H) override { H.resize(0, 0); }

  void clearCache() override {}

  /**
   * Returns a reference to SLQ
   *
   * @return a reference to SLQ
   */
  slq_t& getSLQ() { return *slqPtr_; }

  /**
   * Returns a reference to GDDP
   *
   * @return a reference to GDDP
   */
  gddp_t& getGDDP() { return *gddpPtr_; }

 private:
  std::unique_ptr<slq_t> slqPtr_;
  std::unique_ptr<slq_data_collector_t> slqDataCollectorPtr_;
  std::unique_ptr<gddp_t> gddpPtr_;
  bool display_;

  scalar_t initTime_;
  state_vector_t initState_;
  scalar_t finalTime_;
  scalar_array_t partitioningTimes_;

  scalar_array_t eventTimes_;

  bool status_;
  scalar_t cost_;
  scalar_t constraintISE1_;
  scalar_t constraintISE2_;
};

}  // namespace ocs2
