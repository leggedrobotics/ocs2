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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
NumGDDP<STATE_DIM, INPUT_DIM>::NumGDDP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                       const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                       const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                       const SLQ_Settings& settings /*= SLQ_Settings()*/, const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
                                       const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
           logicRulesPtr, heuristicsFunctionPtr),
      eps_(std::sqrt(settings.ddpSettings_.minRelCost_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
void NumGDDP<STATE_DIM, INPUT_DIM>::getCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {
  // refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
  const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void NumGDDP<STATE_DIM, INPUT_DIM>::setSolverEventTime(const scalar_array_t& eventTimes) {
  BASE::getLogicRulesPtr()->setEventTimes(eventTimes);
  BASE::getLogicRulesMachinePtr()->logicRulesUpdated();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void NumGDDP<STATE_DIM, INPUT_DIM>::run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                        const scalar_array_t& partitioningTimes, const scalar_array_t& eventTimes) {
  // find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
  activeEventTimeBeginIndex_ = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes, initTime));
  activeEventTimeEndIndex_ = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes, finalTime));

  // set the event time
  setSolverEventTime(eventTimes);

  // run SLQ
  BASE::run(initTime, initState, finalTime, partitioningTimes);

  // get cost function
  scalar_t costFunction, constraint1ISE, constraint2ISE;
  BASE::getPerformanceIndeces(costFunction, constraint1ISE, constraint2ISE);

  // get controller for warm starting
  typename BASE::controller_ptr_array_t controller = BASE::getController();

  // numerical computation of the gradient
  nominalCostFuntionDerivative_ = dynamic_vector_t::Zero(eventTimes.size());
  for (size_t i = activeEventTimeBeginIndex_; i < activeEventTimeEndIndex_; i++) {
    // perturbation
    scalar_array_t eventTimesPlus = eventTimes;
    scalar_t h = eps_ * std::max(std::fabs(eventTimesPlus[i]), 1.0);
    eventTimesPlus[i] += h;

    // set the event time
    setSolverEventTime(eventTimesPlus);

    // run SLQ
    BASE::reset();
    BASE::run(initTime, initState, finalTime, partitioningTimes);

    // get cost function
    scalar_t costFunctionPlus, constraint1ISEPlus, constraint2ISEPlus;
    BASE::getPerformanceIndeces(costFunctionPlus, constraint1ISEPlus, constraint2ISEPlus);

    nominalCostFuntionDerivative_[i] = (costFunctionPlus - costFunction) / h;
  }
}

}  // namespace ocs2
