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
GLQP<STATE_DIM, INPUT_DIM>::GLQP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                 const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                 const SLQ_Settings& settings /*= SLQ_Settings()*/,
                                 std::shared_ptr<HybridLogicRules> logicRules /*= nullptr*/,
                                 const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : settings_(settings), logicRulesMachine_(logicRules), numPartitions_(0) {
  // Dynamics, derivatives, and cost
  systemDynamicsPtrStock_.clear();
  systemDynamicsPtrStock_.reserve(settings_.nThreads_);
  systemDerivativesPtrStock_.clear();
  systemDerivativesPtrStock_.reserve(settings_.nThreads_);
  costFunctionsPtrStock_.clear();
  costFunctionsPtrStock_.reserve(settings_.nThreads_);
  heuristicsFunctionsPtrStock_.clear();
  heuristicsFunctionsPtrStock_.reserve(settings_.nThreads_);
  systemEventHandlersPtrStock_.clear();
  systemEventHandlersPtrStock_.reserve(settings_.nThreads_);
  dynamicsIntegratorsStockPtr_.clear();
  dynamicsIntegratorsStockPtr_.reserve(settings_.nThreads_);

  // initialize all subsystems, etc.
  for (size_t i = 0; i < settings_.nThreads_; i++) {
    // initialize dynamics
    systemDynamicsPtrStock_.emplace_back(systemDynamicsPtr->clone());

    // initialize linearized systems
    systemDerivativesPtrStock_.emplace_back(systemDerivativesPtr->clone());

    // initialize cost functions
    costFunctionsPtrStock_.emplace_back(costFunctionPtr->clone());

    // initialize operating trajectories
    operatingTrajectoriesPtrStock_.emplace_back(operatingTrajectoriesPtr->clone());

    // initialize heuristics functions
    if (heuristicsFunctionPtr != nullptr)
      heuristicsFunctionsPtrStock_.emplace_back(heuristicsFunctionPtr->clone());
    else  // use the cost function if no heuristics function is defined
      heuristicsFunctionsPtrStock_.emplace_back(costFunctionPtr->clone());

    // initialize events
    using event_handler_alloc_t = Eigen::aligned_allocator<event_handler_t>;
    systemEventHandlersPtrStock_.push_back(
        std::move(std::allocate_shared<event_handler_t, event_handler_alloc_t>(event_handler_alloc_t())));

    // initialize integrators
    using ode_t = ODE45<STATE_DIM>;
    using ode_alloc_t = Eigen::aligned_allocator<ode_t>;
    dynamicsIntegratorsStockPtr_.push_back(std::move(
        std::allocate_shared<ode_t, ode_alloc_t>(ode_alloc_t(), systemDynamicsPtrStock_.back(), systemEventHandlersPtrStock_.back())));

  }  // end of i loop

  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(settings_.nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(settings_.nThreads_);

  for (size_t i = 0; i < settings_.nThreads_; i++) {
    using riccati_equations_alloc_t = Eigen::aligned_allocator<riccati_equations_t>;
    riccatiEquationsPtrStock_.push_back(std::move(
        std::allocate_shared<riccati_equations_t, riccati_equations_alloc_t>(riccati_equations_alloc_t(), settings_.useMakePSD_)));

    switch (settings_.RiccatiIntegratorType_) {
      case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ODE45: {
        riccatiIntegratorPtrStock_.emplace_back(new ODE45<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back()));
        break;
      }
      /*note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
      case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ADAMS_BASHFORTH: {
        throw std::runtime_error("This ADAMS_BASHFORTH is not implemented for Riccati Integrator.");
        break;
      }
      case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::BULIRSCH_STOER: {
        riccatiIntegratorPtrStock_.emplace_back(new IntegratorBulirschStoer<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back()));
        break;
      }
      default:
        throw(std::runtime_error("Riccati equation integrator type specified wrongly."));
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::findOperatingPointsWorker(size_t workerIndex, const size_t& partitionIndex, const scalar_t& initTime,
                                                           const state_vector_t& initState, const scalar_t& finalTime,
                                                           const scalar_array_t& eventTimes, scalar_array_t& timeOperatingPointsStock,
                                                           size_array_t& eventsPastTheEndIndeces,
                                                           state_vector_array_t& stateOperatingPoints,
                                                           input_vector_array_t& inputOperatingPoints) {
  // max number of steps for integration
  size_t maxNumSteps = 2 * (eventTimes.size() + 1);

  // clearing the output trajectories
  timeOperatingPointsStock.clear();
  timeOperatingPointsStock.reserve(maxNumSteps);
  stateOperatingPoints.clear();
  stateOperatingPoints.reserve(maxNumSteps);
  inputOperatingPoints.clear();
  inputOperatingPoints.reserve(maxNumSteps);
  eventsPastTheEndIndeces.resize(eventTimes.size());

  state_vector_t beginState = initState;
  scalar_t beginTime, endTime;
  size_t k_u = 0;

  for (size_t i = 0; i <= eventTimes.size(); i++) {
    beginTime = (i == 0 ? initTime : eventTimes[i - 1]);
    endTime = (i == eventTimes.size() ? finalTime : eventTimes[i]);

    // skip if finalTime==eventTimes.back()
    // this is consistent with HybridLogicRulesMachine::findEventsDistribution method
    if (i == eventTimes.size() && eventTimes.empty() == false)
      if (std::abs(finalTime - eventTimes.back()) < OCS2NumericTraits<double>::limit_epsilon()) continue;

    // get operating trajectories
    operatingTrajectoriesPtrStock_[workerIndex]->getSystemOperatingTrajectories(beginState, beginTime, endTime, timeOperatingPointsStock,
                                                                                stateOperatingPoints, inputOperatingPoints, true);

    if (i < eventTimes.size()) {
      eventsPastTheEndIndeces[i] = stateOperatingPoints.size();
      beginState = stateOperatingPoints.back();
    }

  }  // end of i loop
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::findOperatingPoints(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                                     const scalar_array_t& partitioningTimes,
                                                     std::vector<scalar_array_t>& timeOperatingPointsStock,
                                                     std::vector<size_array_t>& eventsPastTheEndIndecesStock,
                                                     state_vector_array2_t& stateOperatingPointsStock,
                                                     input_vector_array2_t& inputOperatingPointsStock, size_t threadId /*= 0*/) {
  size_t numPartitions = partitioningTimes.size() - 1;

  timeOperatingPointsStock.resize(numPartitions);
  eventsPastTheEndIndecesStock.resize(numPartitions);
  stateOperatingPointsStock.resize(numPartitions);
  inputOperatingPointsStock.resize(numPartitions);

  // finding the active subsystem index at initTime
  size_t initActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, initTime);
  // finding the active subsystem index at initTime
  size_t finalActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, finalTime);

  scalar_t t0 = initTime;
  state_vector_t x0 = initState;
  scalar_t tf;
  for (size_t i = 0; i < numPartitions; i++) {
    // for subsystems before the initial time
    if (i < initActiveSubsystemIndex || i > finalActiveSubsystemIndex) {
      timeOperatingPointsStock[i].clear();
      eventsPastTheEndIndecesStock[i].clear();
      stateOperatingPointsStock[i].clear();
      inputOperatingPointsStock[i].clear();
      continue;
    }

    // final time
    tf = (i != finalActiveSubsystemIndex) ? partitioningTimes[i + 1] : finalTime;

    // use Base rolloutWorker to use the worker threadId
    findOperatingPointsWorker(threadId, i, t0, x0, tf, logicRulesMachine_.getSwitchingTimes(i), timeOperatingPointsStock[i],
                              eventsPastTheEndIndecesStock[i], stateOperatingPointsStock[i], inputOperatingPointsStock[i]);

    // reset the initial time and state
    t0 = timeOperatingPointsStock[i].back();
    x0 = stateOperatingPointsStock[i].back();

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::rollout(const state_vector_t& initState,
//		const controller_array_t& controllersStock,
//		std::vector<scalar_array_t>& timeTrajectoriesStock,
//		state_vector_array2_t& stateTrajectoriesStock,
//		input_vector_array2_t& inputTrajectoriesStock)  {
//
//	if (controllersStock.size() != numPartitions_)
//		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
//
//	timeTrajectoriesStock.resize(numPartitions_);
//	stateTrajectoriesStock.resize(numPartitions_);
//	inputTrajectoriesStock.resize(numPartitions_);
//
//	state_vector_t x0 = initState;
//	for (int i=0; i<numPartitions_; i++) {
//
//		timeTrajectoriesStock[i].clear();
//		stateTrajectoriesStock[i].clear();
//
//		// set controller for subsystem i
//		systemDynamicsPtrStock_[i]->setController(controllersStock[i]);
//		// simulate subsystem i
//		dynamicsIntegratorsStockPtr_[i]->integrate(x0, partitioningTimes_[i], partitioningTimes_[i+1], stateTrajectoriesStock[i],
// timeTrajectoriesStock[i], 1e-3);
//
//		// compute control trajectory for subsystem i
//		inputTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
//		for (int k=0; k<timeTrajectoriesStock[i].size(); k++)   {
//			systemDynamicsPtrStock_[i]->computeInput(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k],
// inputTrajectoriesStock[i][k]);
//		}
//
//		// reset the initial state
//		x0 = stateTrajectoriesStock[i].back();
//
//		if (x0 != x0)  throw std::runtime_error("The rollout in GLQP is unstable.");
//	}
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
//		const state_vector_array2_t& stateTrajectoriesStock,
//		const input_vector_array2_t& inputTrajectoriesStock,
//		scalar_t& totalCost)  {
//
//	totalCost = 0.0;
//	for (int i=0; i<numPartitions_; i++) {
//
//		scalar_t currentIntermediateCost;
//		scalar_t nextIntermediateCost;
//		for (int k=0; k<timeTrajectoriesStock[i].size()-1; k++) {
//
//			if (k==0) {
//				costFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k],
// stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
// costFunctionsPtrStock_[i]->evaluate(currentIntermediateCost); 			} else {
// currentIntermediateCost = nextIntermediateCost;
//			}
//
//			// feed next state and control to cost function
//			costFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k+1], stateTrajectoriesStock[i][k+1],
// inputTrajectoriesStock[i][k+1]);
//			// evaluate intermediate cost for next time step
//			costFunctionsPtrStock_[i]->evaluate(nextIntermediateCost);
//
//			totalCost +=
// 0.5*(currentIntermediateCost+nextIntermediateCost)*(timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);
//		}
//
//		// terminal cost
//		if (i==numPartitions_-1)  {
//			scalar_t finalCost;
//			costFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i].back(),
// stateTrajectoriesStock[i].back(), inputTrajectoriesStock[i].back());
// costFunctionsPtrStock_[i]->terminalCost(finalCost);
// totalCost += finalCost;
//		}
//	}
//
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
  for (size_t i = 0; i < numPartitions_; i++) {
    // intermediate LQ variables
    size_t N = stateOperatingPointsStock_[i].size();

    AmStock_[i].resize(N);
    BmStock_[i].resize(N);

    qStock_[i].resize(N);
    QvStock_[i].resize(N);
    QmStock_[i].resize(N);
    RvStock_[i].resize(N);
    RmStock_[i].resize(N);
    RmInverseStock_[i].resize(N);
    PmStock_[i].resize(N);

    // switching times LQ variables
    qFinalStock_[i].resize(N - 1);
    QvFinalStock_[i].resize(N - 1);
    QmFinalStock_[i].resize(N - 1);

    if (N > 0) {
      for (size_t j = 0; j < settings_.nThreads_; j++) {
        // set desired trajectories
        costFunctionsPtrStock_[j]->setCostNominalTrajectoriesPtr(desiredTimeTrajectoryPtrStock_[i], desiredStateTrajectoryPtrStock_[i],
                                                                 desiredInputTrajectoryPtrStock_[i]);
      }  // end of j loop

      /*
       * perform the approximateSubsystemLQ for partition i
       */
      approximatePartitionLQ(i);
    }

  }  // end of i loop

  // calculate the Heuristics function at the final time
  heuristicsFunctionsPtrStock_[0]->setCurrentStateAndControl(timeOperatingPointsStock_[finalActivePartition_].back(),
                                                             stateOperatingPointsStock_[finalActivePartition_].back(),
                                                             inputOperatingPointsStock_[finalActivePartition_].back());
  heuristicsFunctionsPtrStock_[0]->terminalCost(sHeuristics_(0));
  heuristicsFunctionsPtrStock_[0]->terminalCostStateDerivative(SvHeuristics_);
  heuristicsFunctionsPtrStock_[0]->terminalCostStateSecondDerivative(SmHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::approximatePartitionLQ(const size_t& partitionIndex) {
  const size_t threadId = 0;
  size_t N = timeOperatingPointsStock_[partitionIndex].size();

  if (N > 0) {
    for (size_t k = 0; k < N; k++) {
      // execute approximateLQWorker for the given partition and time node index (k)
      approximateLQWorker(threadId, partitionIndex, k);
    }  // end of ke loop
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::approximateLQWorker(size_t workerIndex, const size_t& partitionIndex, const size_t& timeIndex) {
  const size_t& i = partitionIndex;
  const size_t& k = timeIndex;

  /*
   * LINEARIZE SYSTEM DYNAMICS
   */

  // set data
  systemDerivativesPtrStock_[workerIndex]->setCurrentStateAndControl(timeOperatingPointsStock_[i][k], stateOperatingPointsStock_[i][k],
                                                                     inputOperatingPointsStock_[i][k]);

  // get results
  systemDerivativesPtrStock_[workerIndex]->getDerivativeState(AmStock_[i][k]);
  systemDerivativesPtrStock_[workerIndex]->getDerivativesControl(BmStock_[i][k]);

  /*
   * QUADRATIC APPROXIMATION TO THE COST FUNCTION
   */

  // set data
  costFunctionsPtrStock_[workerIndex]->setCurrentStateAndControl(timeOperatingPointsStock_[i][k], stateOperatingPointsStock_[i][k],
                                                                 inputOperatingPointsStock_[i][k]);

  // get results
  costFunctionsPtrStock_[workerIndex]->evaluate(qStock_[i][k](0));
  costFunctionsPtrStock_[workerIndex]->stateDerivative(QvStock_[i][k]);
  costFunctionsPtrStock_[workerIndex]->stateSecondDerivative(QmStock_[i][k]);
  costFunctionsPtrStock_[workerIndex]->controlDerivative(RvStock_[i][k]);
  costFunctionsPtrStock_[workerIndex]->controlSecondDerivative(RmStock_[i][k]);
  RmInverseStock_[i][k] = RmStock_[i][k].llt().solve(input_matrix_t::Identity());
  costFunctionsPtrStock_[workerIndex]->stateControlDerivative(PmStock_[i][k]);

  // making sure that constrained Qm is PSD
  if (settings_.useMakePSD_ == true) makePSD(QmStock_[i][k]);

  if (settings_.displayInfo_ && k % 2 == 1) {
    std::cout << "stateOperatingPoint[" << i << "][" << k << "]: \n" << stateOperatingPointsStock_[i][k].transpose() << std::endl;
    std::cout << "inputOperatingPoint[" << i << "][" << k << "]: \n" << inputOperatingPointsStock_[i][k].transpose() << std::endl;
    std::cout << "A[" << i << "][" << k << "]: \n" << AmStock_[i][k] << std::endl;
    std::cout << "B[" << i << "][" << k << "]: \n" << BmStock_[i][k] << std::endl;
    std::cout << "q[" << i << "][" << k << "]: \t" << qStock_[i][k] << std::endl;
    std::cout << "Qv[" << i << "][" << k << "]: \n" << QvStock_[i][k].transpose() << std::endl;
    std::cout << "Qm[" << i << "][" << k << "]: \n" << QmStock_[i][k] << std::endl;
    std::cout << "Rv[" << i << "][" << k << "]: \n" << RvStock_[i][k].transpose() << std::endl;
    std::cout << "Rm[" << i << "][" << k << "]: \n" << RmStock_[i][k] << std::endl;
    std::cout << "Pm[" << i << "][" << k << "]: \n" << PmStock_[i][k] << std::endl;
  }

  // if a switch took place calculate switch related variables
  size_t NE = eventsPastTheEndIndecesStock_[i].size();
  for (size_t ke = 0; ke < NE; ke++) {
    if (eventsPastTheEndIndecesStock_[i][ke] == k + 1) {
      /*
       * Final cost
       */
      costFunctionsPtrStock_[workerIndex]->terminalCost(qFinalStock_[i][ke](0));
      costFunctionsPtrStock_[workerIndex]->terminalCostStateDerivative(QvFinalStock_[i][ke]);
      costFunctionsPtrStock_[workerIndex]->terminalCostStateSecondDerivative(QmFinalStock_[i][ke]);

      // making sure that Qm remains PSD
      if (settings_.useMakePSD_ == true) makePSD(QmFinalStock_[i][ke]);

      if (settings_.displayInfo_) {
        std::cout << "qFinal[" << i << "][" << k << "]: \t" << qFinalStock_[i][ke] << std::endl;
        std::cout << "QvFinal[" << i << "][" << k << "]: \n" << QvFinalStock_[i][ke].transpose() << std::endl;
        std::cout << "QmFinal[" << i << "][" << k << "]: \n" << QmFinalStock_[i][ke] << std::endl;
      }

      break;
    }
  }  // end of ke loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::calculateController(const scalar_t& learningRate, controller_array_t& controllersStock) {
//
//	for (int i=0; i<numPartitions_; i++) {
//
//		controllersStock[i].time_ = SsTimeTrajectoryStock_[i];
//
//		controllersStock[i].k_.resize(SsTimeTrajectoryStock_[i].size());
//		controllersStock[i].uff_.resize(SsTimeTrajectoryStock_[i].size());
//		for (int k=0; k<SsTimeTrajectoryStock_[i].size(); k++) {
//
//			controllersStock[i].k_[k]    = -RmInverseStock_[i] * (PmStock_[i] +
// BmStock_[i].transpose()*SmTrajectoryStock_[i][k]); 			controllersStock[i].uff_[k]  = -learningRate * RmInverseStock_[i] *
// (RvStock_[i]  + BmStock_[i].transpose()*SvTrajectoryStock_[i][k])
//								+ inputOperatingPointsStock_[i] -
// controllersStock[i].k_[k]*stateOperatingPointsStock_[i];
//		}
//
//		if (settings_.displayInfo_ ) {
//			std::cout << "Controller of subsystem" << i << ":" << std::endl;
//			std::cout << "learningRate " << learningRate << std::endl;
//			std::cout << "time: " << controllersStock[i].time_.front() << std::endl;
//			std::cout << "delta_uff: " <<  (controllersStock[i].uff_[0] +
// controllersStock[i].k_[0]*stateOperatingPointsStock_[i]).transpose() << std::endl; 			std::cout << "u0: " <<
// inputOperatingPointsStock_[i].transpose() << std::endl; 			std::cout << "k: \n" <<  controllersStock[i].k_.front() <<
// std::endl << std::endl;
//		}
//	}
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::transformeLocalValueFuntion2Global() {
//
//	for (int i=0; i<numPartitions_; i++)
//		for (int k=0; k<SsTimeTrajectoryStock_[i].size(); k++) {
//
//			sTrajectoryStock_[i][k] = sTrajectoryStock_[i][k] -
// stateOperatingPointsStock_[i].transpose()*SvTrajectoryStock_[i][k]
//+ 0.5*stateOperatingPointsStock_[i].transpose()*SmTrajectoryStock_[i][k]*stateOperatingPointsStock_[i]; SvTrajectoryStock_[i][k] =
// SvTrajectoryStock_[i][k] - SmTrajectoryStock_[i][k]*stateOperatingPointsStock_[i];
//		}
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t GLQP<STATE_DIM, INPUT_DIM>::findActiveSubsystemIndex(const scalar_array_t& partitioningTimes, const double& time) {
  int activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);

  if (activeSubsystemIndex < 0) {
    std::string mesg = "Given time is less than the strat time (i.e. givenTime < partitioningTimes.front()): " + std::to_string(time) +
                       " < " + std::to_string(partitioningTimes.front());
    throw std::runtime_error(mesg);
  }

  if (activeSubsystemIndex == partitioningTimes.size() - 1) {
    std::string mesg = "Given time is greater than the final time (i.e. partitioningTimes.back() < givenTime): " +
                       std::to_string(partitioningTimes.back()) + " < " + std::to_string(time);
    throw std::runtime_error(mesg);
  }

  return (size_t)activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
bool GLQP<STATE_DIM, INPUT_DIM>::makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {
  if (squareMatrix.rows() != squareMatrix.cols()) throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

  Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix);
  Eigen::VectorXd lambda = eig.eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.size(); j++)
    if (lambda(j) < 0.0) {
      hasNegativeEigenValue = true;
      lambda(j) = 0.0;
    }

  if (hasNegativeEigenValue)
    squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  else
    squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();

  return hasNegativeEigenValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::getController(controller_array_t& controllersStock) {
  controllersStock = controllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {
//
//	int activeSubsystem = -1;
//	for (int i=0; i<numPartitions_; i++)  {
//		activeSubsystem = i;
//		if (partitioningTimes_[i]<=time && time<partitioningTimes_[i+1])
//			break;
//	}
//
//	state_matrix_t Sm;
//	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc(&SsTimeTrajectoryStock_[activeSubsystem],
//&SmTrajectoryStock_[activeSubsystem]); 	SmFunc.interpolate(time, Sm); 	state_vector_t Sv;
//	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc(&SsTimeTrajectoryStock_[activeSubsystem],
//&SvTrajectoryStock_[activeSubsystem]); 	SvFunc.interpolate(time, Sv); 	eigen_scalar_t s;
//	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > sFunc(&SsTimeTrajectoryStock_[activeSubsystem],
//&sTrajectoryStock_[activeSubsystem]); 	sFunc.interpolate(time, s);
//
//	valueFuntion = (s + state.transpose()*Sv + 0.5*state.transpose()*Sm*state).eval()(0);
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// template <size_t STATE_DIM, size_t INPUT_DIM>
// void GLQP<STATE_DIM, INPUT_DIM>::solveRiccatiEquations()  {
//
//	SsTimeTrajectoryStock_.resize(numPartitions_);
//	sTrajectoryStock_.resize(numPartitions_);
//	SvTrajectoryStock_.resize(numPartitions_);
//	SmTrajectoryStock_.resize(numPartitions_);
//
//	// final value for the last Riccati equations
//	Eigen::Matrix<double,riccati_equations_t::S_DIM_,1> allSsFinal;
//	riccati_equations_t::convert2Vector(QmFinal_, QvFinal_, qFinal_, allSsFinal);
//
//	for (int i=numPartitions_-1; i>=0; i--) {
//
//		// set data for Riccati equations
//		auto riccatiEquationsPtr = std::allocate_shared< riccati_equations_t,
//				Eigen::aligned_allocator<riccati_equations_t>>(
//						Eigen::aligned_allocator<riccati_equations_t>() );
//
//		riccatiEquationsPtr->setData(partitioningTimes_[i], partitioningTimes_[i+1],
//				AmStock_[i], BmStock_[i],
//				qStock_[i], QvStock_[i], QmStock_[i], RvStock_[i], RmStock_[i], PmStock_[i]);
//
//		// integrating the Riccati equations
//		ODE45<riccati_equations_t::S_DIM_> ode45(riccatiEquationsPtr);
//		scalar_array_t normalizedTimeTrajectory;
//		std::vector<Eigen::Matrix<double,riccati_equations_t::S_DIM_,1>,
// Eigen::aligned_allocator<Eigen::Matrix<double,riccati_equations_t::S_DIM_,1>> > allSsTrajectory; 		ode45.integrate(allSsFinal,
// i, i+1, allSsTrajectory, normalizedTimeTrajectory);
//
//		// denormalizing time and constructing 'Sm', 'Sv', and 's'
//		int N = normalizedTimeTrajectory.size();
//		SsTimeTrajectoryStock_[i].resize(N);
//		SmTrajectoryStock_[i].resize(N);
//		SvTrajectoryStock_[i].resize(N);
//		sTrajectoryStock_[i].resize(N);
//		for (int k=0; k<N; k++) {
//
//			riccati_equations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[i][k], SvTrajectoryStock_[i][k],
// sTrajectoryStock_[i][k]); 			SsTimeTrajectoryStock_[i][k] =
//(partitioningTimes_[i]-partitioningTimes_[i+1])*(normalizedTimeTrajectory[N-1-k]-i) + partitioningTimes_[i+1];
//		}
//
//		// testing the numerical stability of the Riccati equations
//		for (int k=N-1; k>=0; k--) {
//			try {
//				if (SmTrajectoryStock_[i][k] != SmTrajectoryStock_[i][k])  throw std::runtime_error("Sm is unstable");
//				if (SvTrajectoryStock_[i][k] != SvTrajectoryStock_[i][k])  throw std::runtime_error("Sv is unstable");
//				if (sTrajectoryStock_[i][k] != sTrajectoryStock_[i][k])    throw std::runtime_error("s is unstable");
//			}
//			catch(std::exception const& error)
//			{
//				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[i][k] << " [sec]." <<
// std::endl; 				for (int kp=k; kp<k+10; kp++)  { 					if (kp >= N) continue;
// std::cerr << "Sm[" << SsTimeTrajectoryStock_[i][kp] << "]:\n"<< SmTrajectoryStock_[i][kp].transpose() << std::endl;
// std::cerr << "Sv[" << SsTimeTrajectoryStock_[i][kp] << "]:\t"<< SvTrajectoryStock_[i][kp].transpose() << std::endl;
// std::cerr << "s[" << SsTimeTrajectoryStock_[i][kp] << "]: \t"<< sTrajectoryStock_[i][kp].transpose() << std::endl;
//				}
//				exit(1);
//			}
//		}
//
//		// reset the final value for next Riccati equation
//		allSsFinal = allSsTrajectory.back();
//
//		if (allSsFinal != allSsFinal)
//			throw std::runtime_error("Riccati Equation solver in GLQP is unstable.");
//
////		std::cout << "allSsFinal " << i << ":\n" << allSsFinal.transpose() << std::endl;
//	}
//
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::solveRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex,
                                                             const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                                             const eigen_scalar_t& sFinal) {
  // Normalized start and final time for reccati equation
  scalar_t stratNormalizedTime = 0.0;
  scalar_t finalNormalizedTime = 1.0;
  if (partitionIndex == initActivePartition_) {
    finalNormalizedTime = (initTime_ - partitioningTimes_[partitionIndex + 1]) /
                          (partitioningTimes_[partitionIndex] - partitioningTimes_[partitionIndex + 1]);
  } else if (partitionIndex == finalActivePartition_) {
    stratNormalizedTime = (finalTime_ - partitioningTimes_[partitionIndex + 1]) /
                          (partitioningTimes_[partitionIndex] - partitioningTimes_[partitionIndex + 1]);
  }

  // Normalized event times
  size_t NE = logicRulesMachine_.getSwitchingTimes(partitionIndex).size();
  scalar_array_t SsNormalizedEventTimes(NE);
  for (size_t k = 0; k < NE; k++) {
    const scalar_t& eventTime = logicRulesMachine_.getSwitchingTimes(partitionIndex).at(k);
    SsNormalizedEventTimes[NE - 1 - k] = (eventTime - partitioningTimes_[partitionIndex + 1]) /
                                         (partitioningTimes_[partitionIndex] - partitioningTimes_[partitionIndex + 1]);
  }

  // max number of steps of integration
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  size_t maxNumSteps = settings_.maxNumStepsPerSecond_ * std::max(1.0, finalNormalizedTime - stratNormalizedTime);

  // final value for the last Riccati equations plus final cost
  typename riccati_equations_t::s_vector_t allSsFinal;
  riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // clear output containers
  SsNormalizedTimeTrajectoryStock_[partitionIndex].clear();
  SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
  typename riccati_equations_t::s_vector_array_t allSsTrajectory;

  scalar_t beginTime, endTime;
  for (size_t i = 0; i <= NE; i++) {
    beginTime = (i == 0 ? stratNormalizedTime : SsNormalizedEventTimes[i - 1]);
    endTime = (i == NE ? finalNormalizedTime : SsNormalizedEventTimes[i]);

    // set data for Riccati equations
    riccatiEquationsPtrStock_[workerIndex]->reset();
    riccatiEquationsPtrStock_[workerIndex]->setData(
        partitioningTimes_[partitionIndex], partitioningTimes_[partitionIndex + 1], AmStock_[partitionIndex][2 * i],
        BmStock_[partitionIndex][2 * i], qStock_[partitionIndex][2 * i], QvStock_[partitionIndex][2 * i], QmStock_[partitionIndex][2 * i],
        RvStock_[partitionIndex][2 * i], RmStock_[partitionIndex][2 * i], PmStock_[partitionIndex][2 * i]);

    // skip if finalTime==eventTimes.back().
    // This is consistent with HybridLogicRulesMachine::findEventsDistribution method
    if (i == 0 && NE > 0 && std::abs(stratNormalizedTime - SsNormalizedEventTimes.front()) < OCS2NumericTraits<double>::limit_epsilon()) {
      SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][0] = 0;
      typename riccati_equations_t::s_vector_t allSsFinalTemp = allSsFinal;
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(endTime, allSsFinalTemp, allSsFinal);
      continue;
    }

    // solve Riccati equations
    riccatiIntegratorPtrStock_[workerIndex]->integrate(allSsFinal, beginTime, endTime, allSsTrajectory,
                                                       SsNormalizedTimeTrajectoryStock_[partitionIndex], settings_.minTimeStep_,
                                                       settings_.absTolODE_, settings_.relTolODE_, maxNumSteps, true);

    if (i < NE) {
      SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][i] = allSsTrajectory.size();
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(endTime, allSsTrajectory.back(), allSsFinal);
    }

  }  // end of i loop

  // denormalizing time and constructing 'Sm', 'Sv', and 's'
  size_t N = SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
  SsTimeTrajectoryStock_[partitionIndex].resize(N);
  SmTrajectoryStock_[partitionIndex].resize(N);
  SvTrajectoryStock_[partitionIndex].resize(N);
  sTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    riccati_equations_t::convert2Matrix(allSsTrajectory[N - 1 - k], SmTrajectoryStock_[partitionIndex][k],
                                        SvTrajectoryStock_[partitionIndex][k], sTrajectoryStock_[partitionIndex][k]);
    SsTimeTrajectoryStock_[partitionIndex][k] = (partitioningTimes_[partitionIndex] - partitioningTimes_[partitionIndex + 1]) *
                                                    SsNormalizedTimeTrajectoryStock_[partitionIndex][N - 1 - k] +
                                                partitioningTimes_[partitionIndex + 1];
  }  // end of k loop

  // testing the numerical stability of the Riccati equations
  if (settings_.checkNumericalStability_)
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (SmTrajectoryStock_[partitionIndex][k].hasNaN()) throw std::runtime_error("Sm is unstable.");
        if (SvTrajectoryStock_[partitionIndex][k].hasNaN()) throw std::runtime_error("Sv is unstable.");
        if (sTrajectoryStock_[partitionIndex][k].hasNaN()) throw std::runtime_error("s is unstable.");
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) continue;
          std::cerr << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::solveRiccatiEquations(const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                                       const eigen_scalar_t& sFinal) {
  // for all partition, there is only one worker
  const size_t workerIndex = 0;

  SmFinalStock_[finalActivePartition_ + 1] = SmFinal;
  SvFinalStock_[finalActivePartition_ + 1] = SvFinal;
  sFinalStock_[finalActivePartition_ + 1] = sFinal;

  for (int i = numPartitions_ - 1; i >= 0; i--) {
    if (i < (signed)initActivePartition_ || i > (signed)finalActivePartition_) {
      SsTimeTrajectoryStock_[i].clear();
      SsNormalizedTimeTrajectoryStock_[i].clear();
      SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
      SmTrajectoryStock_[i].clear();
      SvTrajectoryStock_[i].clear();
      sTrajectoryStock_[i].clear();

      SmFinalStock_[i].setZero();
      SvFinalStock_[i].setZero();
      sFinalStock_[i].setZero();

      continue;
    }

    solveRiccatiEquationsWorker(workerIndex, i, SmFinalStock_[i + 1], SvFinalStock_[i + 1], sFinalStock_[i + 1]);

    // set the final value for next Riccati equation
    sFinalStock_[i] = sTrajectoryStock_[i].front();
    SvFinalStock_[i] = SvTrajectoryStock_[i].front();
    SmFinalStock_[i] = SmTrajectoryStock_[i].front();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::setupOptimizer(const size_t& numPartitions) {
  if (numPartitions == 0) throw std::runtime_error("Number of Partitionings cannot be zero!");

  /*
   * Desired cost trajectories
   */
  desiredTimeTrajectoryPtrStock_.resize(numPartitions);
  desiredStateTrajectoryPtrStock_.resize(numPartitions);
  desiredInputTrajectoryPtrStock_.resize(numPartitions);

  timeOperatingPointsStock_.resize(numPartitions);
  eventsPastTheEndIndecesStock_.resize(numPartitions);
  stateOperatingPointsStock_.resize(numPartitions);
  inputOperatingPointsStock_.resize(numPartitions);

  controllersStock_.resize(numPartitions);

  /*
   * approximate LQ variables
   */
  AmStock_.resize(numPartitions);
  BmStock_.resize(numPartitions);

  qFinalStock_.resize(numPartitions);
  QvFinalStock_.resize(numPartitions);
  QmFinalStock_.resize(numPartitions);

  qStock_.resize(numPartitions);
  QvStock_.resize(numPartitions);
  QmStock_.resize(numPartitions);
  RvStock_.resize(numPartitions);
  RmStock_.resize(numPartitions);
  RmInverseStock_.resize(numPartitions);
  PmStock_.resize(numPartitions);

  /*
   * Riccati solver variables and controller update
   */
  SsTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  sTrajectoryStock_.resize(numPartitions);
  SvTrajectoryStock_.resize(numPartitions);
  SmTrajectoryStock_.resize(numPartitions);

  sFinalStock_.resize(numPartitions);
  SvFinalStock_.resize(numPartitions);
  SmFinalStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                     const scalar_array_t& partitioningTimes, const scalar_t& learningRate /*= 1.0*/,
                                     const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock /*= std::vector<scalar_array_t>()*/,
                                     const state_vector_array2_t& desiredStateTrajectoriesStock /*= state_vector_array2_t()*/,
                                     const input_vector_array2_t& desiredInputTrajectoriesStock /*= input_vector_array2_t()*/) {
  // update numPartitions_ if it has been changed
  if (numPartitions_ + 1 != partitioningTimes.size()) {
    numPartitions_ = partitioningTimes.size() - 1;
    partitioningTimes_ = partitioningTimes;
    setupOptimizer(numPartitions_);
  }

  // update partitioningTimes_
  partitioningTimes_ = partitioningTimes;

  // set desired trajectories for cost
  if (desiredTimeTrajectoriesStock.empty() == false) {
    if (desiredTimeTrajectoriesStock.size() != numPartitions_)
      throw std::runtime_error("desiredTimeTrajectoriesStock has less elements than the number of partitions.");
    if (desiredStateTrajectoriesStock.size() != numPartitions_)
      throw std::runtime_error("desiredStateTrajectoriesStock has less elements than the number of partitions.");
    if (desiredInputTrajectoriesStock.size() != numPartitions_ && desiredInputTrajectoriesStock.empty() == false)
      throw std::runtime_error("desiredInputTrajectoriesStock has less elements than the number of partitions.");

    for (size_t i = 0; i < numPartitions_; i++) {
      desiredTimeTrajectoryPtrStock_[i] = std::shared_ptr<const scalar_array_t>(&desiredTimeTrajectoriesStock[i]);
      desiredStateTrajectoryPtrStock_[i] = std::shared_ptr<const state_vector_array_t>(&desiredStateTrajectoriesStock[i]);
      if (desiredInputTrajectoriesStock.empty() == false)
        desiredInputTrajectoryPtrStock_[i] = std::shared_ptr<const input_vector_array_t>(&desiredInputTrajectoriesStock[i]);
      else
        desiredInputTrajectoryPtrStock_[i] = nullptr;
    }  // end of i loop

  } else {
    if (settings_.displayInfo_) std::cerr << "WARNING: Desired trajectories are not provided." << std::endl;
    std::fill(desiredTimeTrajectoryPtrStock_.begin(), desiredTimeTrajectoryPtrStock_.end(), nullptr);
    std::fill(desiredStateTrajectoryPtrStock_.begin(), desiredStateTrajectoryPtrStock_.end(), nullptr);
    std::fill(desiredInputTrajectoryPtrStock_.begin(), desiredInputTrajectoryPtrStock_.end(), nullptr);
  }

  // update the logic rules in the begining of the run routine and adjust the nominal controllerStock based on an user-defined function
  logicRulesMachine_.updateLogicRules(partitioningTimes_, controllersStock_);

  std::cout << "LQ display: " << std::endl;
  logicRulesMachine_.displaySwitchedSystemsDistribution();

  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;

  // the initial active partition index.
  initActivePartition_ = findActiveSubsystemIndex(partitioningTimes_, initTime_);
  // the final active partition index.
  finalActivePartition_ = findActiveSubsystemIndex(partitioningTimes_, finalTime_);

  // find operating points
  findOperatingPoints(initTime_, initState_, finalTime_, partitioningTimes_, timeOperatingPointsStock_, eventsPastTheEndIndecesStock_,
                      stateOperatingPointsStock_, inputOperatingPointsStock_);

  // linearizing the dynamics and quadratizing the cost funtion along nominal trajectories
  approximateOptimalControlProblem();

  // solve Riccati equations
  solveRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);

  //	// calculate controller
  //	controllersStock_.resize(numPartitions_);
  //	calculateController(learningRate, controllersStock_);
  //
  //	// transforme the local value funtion to the global representation
  //	transformeLocalValueFuntion2Global();
}

}  // namespace ocs2
