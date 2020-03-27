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

#include <ocs2_ddp/DDP_DataCollector.h>
#include <ocs2_ddp/SLQ.h>

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

  using BASE = NLP_Cost;

  using gddp_t = GDDP<STATE_DIM, INPUT_DIM>;
  using slq_t = SLQ<STATE_DIM, INPUT_DIM>;
  using ddp_data_collector_t = DDP_DataCollector<STATE_DIM, INPUT_DIM>;

  using state_vector_t = typename slq_t::state_vector_t;
  using derivatives_base_t = typename slq_t::derivatives_base_t;
  using constraint_base_t = typename slq_t::constraint_base_t;
  using cost_function_base_t = typename slq_t::cost_function_base_t;
  using operating_trajectories_base_t = typename slq_t::operating_trajectories_base_t;
  using rollout_base_t = typename slq_t::rollout_base_t;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  UpperLevelCost(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                 const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings,
                 std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr,
                 const cost_function_base_t* heuristicsFunctionPtr = nullptr, bool display = false,
                 GDDP_Settings gddpSettings = GDDP_Settings())
      : slqPtr_(new slq_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
                          heuristicsFunctionPtr)),
        slqDataCollectorPtr_(new ddp_data_collector_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr)),
        modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
        gddpPtr_(new gddp_t(std::move(gddpSettings))),
        display_(display) {
    slqPtr_->setModeScheduleManager(modeScheduleManagerPtr_);
  }

  /**
   * Default destructor.
   */
  ~UpperLevelCost() override = default;

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
    // set mode schedule
    const scalar_array_t eventTimes(x.data(), x.data() + x.size());
    const auto modeSequence = modeScheduleManagerPtr_->getModeSchedule().modeSequence;
    modeScheduleManagerPtr_->setModeSchedule({eventTimes, modeSequence});

    // run SLQ
    try {
      slqPtr_->reset();
      slqPtr_->run(initTime_, initState_, finalTime_, partitioningTimes_);
    } catch (const std::exception& e) {
      std::cerr << "\t     exception: " << e.what();
      return 0;
    }

    // get the cost and constraints ISE
    performanceIndex_ = slqPtr_->getPerformanceIndeces();

    // display
    if (display_) {
      std::cerr << "\t     state equality constraints ISE:       " << performanceIndex_.stateEqConstraintISE
                << "\t     state-input equality constraints ISE: " << performanceIndex_.stateInputEqConstraintISE
                << "\t#Iterations: " << slqPtr_->getNumIterations() << std::endl;
    }

    // collect the SLQ solution
    slqDataCollectorPtr_->collect(slqPtr_.get());

    // status is false if the constraints ISE is higher than minAbsConstraint1RMSE_
    bool status1 = (performanceIndex_.stateInputEqConstraintISE <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;
    bool status2 = (performanceIndex_.stateEqConstraintISE <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;
    status_ = status1 && status2;

    return 0;
  }

  bool getCost(size_t id, scalar_t& f) override {
    f = performanceIndex_.merit;
    return status_;
  }

  void getCostDerivative(size_t id, dynamic_vector_t& g) override {
    gddpPtr_->run(slqDataCollectorPtr_.get());
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
  std::unique_ptr<ddp_data_collector_t> slqDataCollectorPtr_;
  std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr_;
  std::unique_ptr<gddp_t> gddpPtr_;
  bool display_;

  scalar_t initTime_;
  state_vector_t initState_;
  scalar_t finalTime_;
  scalar_array_t partitioningTimes_;

  bool status_;
  PerformanceIndex performanceIndex_;
};

}  // namespace ocs2
