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
OCS2<STATE_DIM, INPUT_DIM>::OCS2(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                 const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings,
                                 std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr,
                                 const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/,
                                 const GDDP_Settings& gddpSettings /*= GDDP_Settings()*/,
                                 const NLP_Settings& nlpSettings /*= NLP_Settings()*/)

    : frankWolfeGradientDescentSolver_(nlpSettings),
      ulCostPtr_(new upper_level_cost_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
                                        settings, std::move(modeScheduleManagerPtr), heuristicsFunctionPtr, nlpSettings.displayInfo_,
                                        gddpSettings)),
      ulConstraintsPtr_(new upper_level_constraints_t) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
GDDP_Settings& OCS2<STATE_DIM, INPUT_DIM>::gddpSettings() {
  return ulCostPtr_->getGDDP().settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& OCS2<STATE_DIM, INPUT_DIM>::slqSettings() {
  return ulCostPtr_->getSLQ().settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
NLP_Settings& OCS2<STATE_DIM, INPUT_DIM>::nlpSettings() {
  return frankWolfeGradientDescentSolver_.nlpSettings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2<STATE_DIM, INPUT_DIM>::getIterationsLog(scalar_array_t& iterationCost) const {
  frankWolfeGradientDescentSolver_.getIterationsLog(iterationCost);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2<STATE_DIM, INPUT_DIM>::getParameters(dynamic_vector_t& parameters) const {
  frankWolfeGradientDescentSolver_.getParameters(parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2<STATE_DIM, INPUT_DIM>::getCost(scalar_t& cost) const {
  frankWolfeGradientDescentSolver_.getCost(cost);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2<STATE_DIM, INPUT_DIM>::run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                     const scalar_array_t& partitioningTimes, const scalar_array_t& initEventTimes) {
  ulCostPtr_->setDDP(initTime, initState, finalTime, partitioningTimes);
  ulConstraintsPtr_->set(initTime, finalTime);

  const size_t numEventTimes = initEventTimes.size();
  dynamic_vector_t initParameters = Eigen::Map<const dynamic_vector_t>(initEventTimes.data(), numEventTimes);

  // run Frank-Wolfe gradient descent
  dynamic_vector_t maxGradientInverse = dynamic_vector_t::Zero(numEventTimes);
  frankWolfeGradientDescentSolver_.run(initParameters, maxGradientInverse, ulCostPtr_.get(), ulConstraintsPtr_.get());
}

}  // namespace ocs2
