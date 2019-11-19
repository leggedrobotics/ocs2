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

#include <ocs2_core/logic/rules/HybridLogicRules.h>
namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
OCS2Projected<STATE_DIM, INPUT_DIM>::OCS2Projected(
    const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
    const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
    const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& slqSettings /*= SLQ_Settings()*/,
    const GDDP_Settings& gddpSettings /*= GDDP_Settings()*/, std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
    const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : slqSettings_(slqSettings) {
  // NLP optimizer options
  BASE::nlpSettings().displayGradientDescent_ = slqSettings_.displayGradientDescent_;
  BASE::nlpSettings().maxIterations_ = slqSettings_.maxIterationGradientDescent_;
  BASE::nlpSettings().minRelCost_ = slqSettings_.acceptableTolGradientDescent_;
  BASE::nlpSettings().maxLearningRate_ = slqSettings_.maxLearningRateNLP_;
  BASE::nlpSettings().minLearningRate_ = slqSettings_.minLearningRateNLP_;
  BASE::nlpSettings().minDisToBoundary_ = slqSettings_.minEventTimeDifference_;
  BASE::nlpSettings().useAscendingLineSearchNLP_ = slqSettings_.useAscendingLineSearchNLP_;
  BASE::adjustOptions();

  // GDDP
  gddpSolverPtr_.reset(new gddp_t(gddpSettings));

  // SLQ data collector
  slqDataCollectorPtr_.reset(new slq_data_collector_t());

  slqSolverPtrs_.resize(BASE::numLineSearch() + 1);
  for (size_t i = 0; i < slqSolverPtrs_.size(); i++) {
    // SLQ solvers
    slqSolverPtrs_[i].reset(new slq_t(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr,
                                      operatingTrajectoriesPtr, slqSettings, logicRulesPtr, heuristicsFunctionPtr));

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getCostFunction(scalar_t& costFunction) const {
  costFunction = optimizedTotalCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getCostFunctionDerivative(dynamic_vector_t& costFuntionDerivative) const {
  costFuntionDerivative = costFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getEventTimes(scalar_array_t& eventTimes) const {
  eventTimes = optimizedEventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getController(controller_array_t& optimizedControllersStock) const {
  optimizedControllersStock = optimizedControllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getNominalTrajectories(std::vector<scalar_array_t>& optimizedTimeTrajectoriesStock,
                                                                 state_vector_array2_t& optimizedStateTrajectoriesStock,
                                                                 input_vector_array2_t& optimizedInputTrajectoriesStock) const {
  optimizedTimeTrajectoriesStock = optimizedTimeTrajectoriesStock_;
  optimizedStateTrajectoriesStock = optimizedStateTrajectoriesStock_;
  optimizedInputTrajectoriesStock = optimizedInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getSLQIterationsLog(eigen_scalar_array_t& slqIterationCost,
                                                              eigen_scalar_array_t& slqIterationISE1,
                                                              eigen_scalar_array_t& slqIterationISE2) const {
  slqIterationCost = slqIterationCost_;
  slqIterationISE1 = slqIterationISE1_;
  slqIterationISE2 = slqIterationISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getOCS2IterationsLog(eigen_scalar_array_t& iterationCost) const {
  iterationCost = iterationCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& OCS2Projected<STATE_DIM, INPUT_DIM>::slqSettings() {
  return slqSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t OCS2Projected<STATE_DIM, INPUT_DIM>::findNearestController(const dynamic_vector_t& enquiryParameter) const {
  if (parameterBag_.size() == 0) {
    throw std::runtime_error("controllerStock bag is empty.");
  }

  // evaluating distance
  std::vector<scalar_t> distance(parameterBag_.size());
  for (size_t i = 0; i < parameterBag_.size(); i++) {
    distance[i] = (parameterBag_[i] - enquiryParameter).squaredNorm();
  }

  // min index
  auto it = std::min_element(distance.begin(), distance.end());

  return it - distance.begin();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::calculateLinearEqualityConstraint(dynamic_matrix_t& Am, dynamic_vector_t& Bv) {
  Am = dynamic_matrix_t::Zero(BASE::numParameters() + 1, BASE::numParameters());
  for (size_t i = 0; i < BASE::numParameters() + 1; i++) {
    if (i < BASE::numParameters()) {
      Am(i, i) = -1.0;
    }
    if (i > 0) {
      Am(i, i - 1) = 1.0;
    }
  }

  Bv = dynamic_vector_t::Zero(BASE::numParameters() + 1);
  Bv(0) = initTime_;
  Bv(BASE::numParameters()) = -finalTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool OCS2Projected<STATE_DIM, INPUT_DIM>::calculateGradient(const size_t& id, const dynamic_vector_t& parameters,
                                                            dynamic_vector_t& gradient) {
  // collect the SLQ data
  slqDataCollectorPtr_->collect(slqSolverPtrs_[id].get());

  // run GSLQ
  gddpSolverPtr_->run(scalar_array_t(parameters.data(), parameters.data() + parameters.size()), slqDataCollectorPtr_.get());

  // get gradient
  gddpSolverPtr_->getCostFuntionDerivative(costFuntionDerivative_);
  gradient = costFuntionDerivative_;

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool OCS2Projected<STATE_DIM, INPUT_DIM>::calculateCost(const size_t& id, const dynamic_vector_t& parameters, scalar_t& cost) {
  // switching time vector
  scalar_array_t eventTimes(BASE::numParameters());
  for (size_t j = 0; j < BASE::numParameters(); j++) {
    eventTimes[j] = parameters(j);
  }

  // TODO
  std::share_ptr<HybridLogicRules> logicRules(new HybridLogicRules());
  logicRules->setEventTimes(eventTimes);
  slqSolverPtrs_[id]->setLogicRules(logicRules);

  // cold or warm start
  bool warmStart = slqSettings_.warmStartGSLQ_ == true && parameterBag_.size() > 0;

  // run SLQ using warm start
  if (warmStart == true) {
    // find most similar controller based on the parameter
    size_t warmStartIndex = findNearestController(parameters);

    // run SLQ
    try {
      slqSolverPtrs_[id]->run(initTime_, initState_, finalTime_, partitioningTimes_, controllersStockBag_[warmStartIndex]);
    } catch (const std::exception& e) {
      std::cerr << "\t     exception: " << e.what();
      //			return false;
      std::cerr << "\t     repeat SLQ with cold start." << std::endl;
      warmStart = false;
    }
  }

  // run SLQ using cold start
  if (warmStart == false) {
    // run SLQ
    try {
      slqSolverPtrs_[id]->run(initTime_, initState_, finalTime_, partitioningTimes_);
    } catch (const std::exception& e) {
      std::cerr << "\t     exception: " << e.what();
      return false;
    }
  }

  // get the cost and constraints ISE
  scalar_t constraintISE1, constraintISE2;
  slqSolverPtrs_[id]->getPerformanceIndeces(cost, constraintISE1, constraintISE2);

  // display
  if (BASE::nlpSettings().displayGradientDescent_) {
    std::cerr << "\t     constraintISE1: " << constraintISE1 << "\t     constraintISE2: " << constraintISE2
              << "\t#Iterations: " << slqSolverPtrs_[id]->getNumIterations() << std::endl;
  }

  // saving solution in the bag
  saveToBag(id, parameters);

  // status is false if the constraints ISE is higher than minAbsConstraint1RMSE_
  bool status1 = (constraintISE1 <= slqSettings_.minAbsConstraint1ISE_) ? true : false;
  bool status2 = (constraintISE2 <= slqSettings_.minAbsConstraint1ISE_) ? true : false;

  return status1 && status2;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::saveToBag(size_t id, const dynamic_vector_t& parameters) {
  // get the nominal trajectories
  const std::vector<scalar_array_t>* timeTrajectoriesStockPtr;
  const state_vector_array2_t* stateTrajectoriesStockPtr;
  const input_vector_array2_t* inputTrajectoriesStockPtr;
  slqSolverPtrs_[id]->getNominalTrajectoriesPtr(timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr);

  // get the optimized controller
  controller_array_t controllersStock = slqSolverPtrs_[id]->getController();

  // save the parameter and controller in the Bag
  parameterBag_.push_back(parameters);
  controllersStockBag_.push_back(std::move(controllersStock));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getSolution(size_t idStar) {
  optimizedEventTimes_ = slqSolverPtrs_[idStar]->getLogicRulesPtr()->eventTimes();
  optimizedControllersStock_ = slqSolverPtrs_[idStar]->getController();
  optimizedTimeTrajectoriesStock_ = slqSolverPtrs_[idStar]->getNominalTimeTrajectories();
  optimizedStateTrajectoriesStock_ = slqSolverPtrs_[idStar]->getNominalStateTrajectories();
  optimizedInputTrajectoriesStock_ = slqSolverPtrs_[idStar]->getNominalInputTrajectories();
  slqSolverPtrs_[idStar]->getPerformanceIndeces(optimizedTotalCost_, optimizedConstraint1ISE_, optimizedConstraint2ISE_);
  slqSolverPtrs_[idStar]->getIterationsLog(slqIterationCost_, slqIterationISE1_, slqIterationISE2_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::rewindOptimizer(const size_t& firstIndex, bool initRun /*=false*/) {
  // rewind SLQ solvers
  for (size_t i = 0; i < slqSolverPtrs_.size(); i++) {
    slqSolverPtrs_[i]->rewindOptimizer(firstIndex, initRun);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::setupOptimizer(const size_t& numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("Number of partitions cannot be zero!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::run(
    const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
    const scalar_array_t& initEventTimes, const cost_desired_trajectories_t& costDesiredTrajectories /*= cost_desired_trajectories_t()*/) {
  // clear the solutions bag
  parameterBag_.clear();
  controllersStockBag_.clear();

  initTime_ = initTime;
  initState_ = initState;
  finalTime_ = finalTime;
  partitioningTimes_ = partitioningTimes;
  initEventTimes_ = initEventTimes;
  numSubsystems_ = initEventTimes.size() + 1;

  if (numPartitions_ != partitioningTimes.size() - 1) {
    numPartitions_ = partitioningTimes.size() - 1;
    setupOptimizer(numPartitions_);
  }

  gddpSolverPtr_->settings() = slqSettings_;

  // for each SLQ solver
  for (size_t i = 0; i < slqSolverPtrs_.size(); i++) {
    // reset solvers
    slqSolverPtrs_[i]->reset();
    // set the settings (they might have been modified after construction)
    slqSolverPtrs_[i]->settings() = slqSettings_;
    // update cost's desired trajectory if it is set
    if (costDesiredTrajectories.empty() == false) {
      slqSolverPtrs_[i]->setCostDesiredTrajectories(costDesiredTrajectories);
    }
  }

  // from std vector to eEigen vector
  dynamic_vector_t initParameters = Eigen::Map<dynamic_vector_t>(initEventTimes_.data(), initEventTimes_.size());

  // run the gradient descent algorithm
  BASE::run(initParameters);
}

}  // namespace ocs2
