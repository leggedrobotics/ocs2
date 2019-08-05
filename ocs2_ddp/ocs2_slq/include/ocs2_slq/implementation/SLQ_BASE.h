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
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_BASE<STATE_DIM, INPUT_DIM>::SLQ_BASE(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                         const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                         const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                         const SLQ_Settings& settings /*= SLQ_Settings()*/,
                                         std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                                         const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           settings.rolloutSettings_, heuristicsFunctionPtr, "SLQ", std::move(logicRulesPtr)),
      settings_(settings) {
  // State triggered
  state_dynamicsForwardRolloutPtrStock_.resize(BASE::ddpSettings_.nThreads_);
  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    state_dynamicsForwardRolloutPtrStock_[i].reset(new state_triggered_rollout_t(*systemDynamicsPtr, BASE::rolloutSettings_, "SLQ"));
  }  // end of i loop

  // for controller design
  BmFunc_.resize(BASE::ddpSettings_.nThreads_);
  PmFunc_.resize(BASE::ddpSettings_.nThreads_);
  RmInverseFunc_.resize(BASE::ddpSettings_.nThreads_);
  RvFunc_.resize(BASE::ddpSettings_.nThreads_);
  EvProjectedFunc_.resize(BASE::ddpSettings_.nThreads_);
  CmProjectedFunc_.resize(BASE::ddpSettings_.nThreads_);
  DmProjectedFunc_.resize(BASE::ddpSettings_.nThreads_);

  // Riccati Solver
  SmFuncs_.resize(BASE::ddpSettings_.nThreads_);
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  errorEquationPtrStock_.clear();
  errorEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  riccatiEventPtrStock_.clear();
  riccatiEventPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  errorEventPtrStock_.clear();
  errorEventPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  errorIntegratorPtrStock_.clear();
  errorIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    using riccati_equations_alloc_t = Eigen::aligned_allocator<riccati_equations_t>;
    riccatiEquationsPtrStock_.emplace_back(std::allocate_shared<riccati_equations_t, riccati_equations_alloc_t>(
        riccati_equations_alloc_t(), BASE::ddpSettings_.useMakePSD_, BASE::ddpSettings_.addedRiccatiDiagonal_,
        settings_.preComputeRiccatiTerms_));

    using error_equation_alloc_t = Eigen::aligned_allocator<error_equation_t>;
    errorEquationPtrStock_.emplace_back(std::allocate_shared<error_equation_t, error_equation_alloc_t>(error_equation_alloc_t()));

    using riccati_event_handler_t = SystemEventHandler<riccati_equations_t::S_DIM_>;
    using riccati_event_handler_alloc_t = Eigen::aligned_allocator<riccati_event_handler_t>;
    riccatiEventPtrStock_.emplace_back(
        std::allocate_shared<riccati_event_handler_t, riccati_event_handler_alloc_t>(riccati_event_handler_alloc_t()));

    using error_event_handler_t = SystemEventHandler<STATE_DIM>;
    using error_event_handler_alloc_t = Eigen::aligned_allocator<error_event_handler_t>;
    errorEventPtrStock_.emplace_back(
        std::allocate_shared<error_event_handler_t, error_event_handler_alloc_t>(error_event_handler_alloc_t()));

    switch (settings_.RiccatiIntegratorType_) {
      case DIMENSIONS::RiccatiIntegratorType::ODE45: {
        riccatiIntegratorPtrStock_.emplace_back(
            new ODE45<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back(), riccatiEventPtrStock_.back()));
        errorIntegratorPtrStock_.emplace_back(new ODE45<STATE_DIM>(errorEquationPtrStock_.back(), errorEventPtrStock_.back()));
        break;
      }
      /*note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
      case DIMENSIONS::RiccatiIntegratorType::ADAMS_BASHFORTH: {
        throw std::runtime_error("This ADAMS_BASHFORTH is not implemented for Riccati Integrator.");
        break;
      }
      case DIMENSIONS::RiccatiIntegratorType::BULIRSCH_STOER: {
        riccatiIntegratorPtrStock_.emplace_back(
            new IntegratorBulirschStoer<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back(), riccatiEventPtrStock_.back()));
        errorIntegratorPtrStock_.emplace_back(
            new IntegratorBulirschStoer<STATE_DIM>(errorEquationPtrStock_.back(), errorEventPtrStock_.back()));
        break;
      }
      default:
        throw(std::runtime_error("Riccati equation integrator type specified wrongly."));
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();
    // resizing the new variables containers
    // for constraints
    DmDagerTrajectoryStock_[i].resize(N);
    AmConstrainedTrajectoryStock_[i].resize(N);
    QmConstrainedTrajectoryStock_[i].resize(N);
    QvConstrainedTrajectoryStock_[i].resize(N);
    EvProjectedTrajectoryStock_[i].resize(N);
    CmProjectedTrajectoryStock_[i].resize(N);
    DmProjectedTrajectoryStock_[i].resize(N);
    RmInverseTrajectoryStock_[i].resize(N);
    RmInvConstrainedCholTrajectoryStock_[i].resize(N);
  }  // end of i loop

  // base method should be called after the resizes
  BASE::approximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  // unconstrained LQ problem
  BASE::approximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

  const scalar_t stateConstraintPenalty =
      BASE::ddpSettings_.stateConstraintPenaltyCoeff_ * pow(BASE::ddpSettings_.stateConstraintPenaltyBase_, BASE::iteration_);

  // modify the unconstrained LQ coefficients to constrained ones
  approximateConstrainedLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);

  // calculate an LQ approximate of the event times process.
  BASE::approximateEventsLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::approximateConstrainedLQWorker(size_t workerIndex, size_t i, size_t k,
                                                                    scalar_t stateConstraintPenalty) {
  // constraint type 2 coefficients
  const auto nc2 = BASE::nc2TrajectoriesStock_[i][k];
  if (nc2 > 0) {
    BASE::qTrajectoryStock_[i][k] +=
        0.5 * stateConstraintPenalty * BASE::HvTrajectoryStock_[i][k].head(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
    BASE::QvTrajectoryStock_[i][k] +=
        stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
    BASE::QmTrajectoryStock_[i][k] +=
        stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::FmTrajectoryStock_[i][k].topRows(nc2);
  }

  // Inequality constraints
  if (BASE::ncIneqTrajectoriesStock_[i][k] > 0) {
    scalar_t p;
    state_vector_t dpdx;
    input_vector_t dpdu;
    state_matrix_t ddpdxdx;
    input_matrix_t ddpdudu;
    input_state_matrix_t ddpdudx;
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCost(BASE::hTrajectoryStock_[i][k], p);
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeState(BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k],
                                                                       dpdx);
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInput(BASE::hTrajectoryStock_[i][k], BASE::dhduTrajectoryStock_[i][k],
                                                                       dpdu);
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeState(
        BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k], BASE::ddhdxdxTrajectoryStock_[i][k], ddpdxdx);
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeInput(
        BASE::hTrajectoryStock_[i][k], BASE::dhduTrajectoryStock_[i][k], BASE::ddhduduTrajectoryStock_[i][k], ddpdudu);
    BASE::penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInputState(BASE::hTrajectoryStock_[i][k], BASE::dhdxTrajectoryStock_[i][k],
                                                                            BASE::dhduTrajectoryStock_[i][k],
                                                                            BASE::ddhdudxTrajectoryStock_[i][k], ddpdudx);
    BASE::qTrajectoryStock_[i][k][0] += p;  // q is a 1x1 matrix, so access it with [0]
    BASE::QvTrajectoryStock_[i][k] += dpdx;
    BASE::QmTrajectoryStock_[i][k] += ddpdxdx;
    BASE::RvTrajectoryStock_[i][k] += dpdu;
    BASE::RmTrajectoryStock_[i][k] += ddpdudu;
    BASE::PmTrajectoryStock_[i][k] += ddpdudx;

    // checking the numerical stability again
    if (BASE::ddpSettings_.checkNumericalStability_) {
      try {
        if (!BASE::qTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost is is not finite.");
        }
        if (!BASE::QvTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost first derivative w.r.t. state is is not finite.");
        }
        if (!BASE::QmTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not finite.");
        }
        if (!BASE::QmTrajectoryStock_[i][k].isApprox(BASE::QmTrajectoryStock_[i][k].transpose())) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not self-adjoint.");
        }
        if (BASE::QmTrajectoryStock_[i][k].eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(BASE::QmTrajectoryStock_[i][k].eigenvalues().real().minCoeff()) + ".");
        }
        if (!BASE::RvTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost first derivative w.r.t. input is is not finite.");
        }
        if (!BASE::RmTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not finite.");
        }
        if (!BASE::RmTrajectoryStock_[i][k].isApprox(BASE::RmTrajectoryStock_[i][k].transpose())) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not self-adjoint.");
        }
        if (!BASE::PmTrajectoryStock_[i][k].allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input-state is is not finite.");
        }
        if (BASE::RmTrajectoryStock_[i][k].ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("R matrix is not invertible. It's reciprocal condition number is " +
                                   std::to_string(BASE::RmTrajectoryStock_[i][k].ldlt().rcond()) + ".");
        }
        if (BASE::RmTrajectoryStock_[i][k].eigenvalues().real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("R matrix is not positive definite. It's smallest eigenvalue is " +
                                   std::to_string(BASE::RmTrajectoryStock_[i][k].eigenvalues().real().minCoeff()) + ".");
        }
      } catch (const std::exception& error) {
        std::cerr << "After adding inequality constraint penalty" << std::endl;
        std::cerr << "what(): " << error.what() << " at time " << BASE::nominalTimeTrajectoriesStock_[i][k] << " [sec]." << std::endl;
        std::cerr << "x: " << BASE::nominalStateTrajectoriesStock_[i][k].transpose() << std::endl;
        std::cerr << "u: " << BASE::nominalInputTrajectoriesStock_[i][k].transpose() << std::endl;
        std::cerr << "q: " << BASE::qTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Qv: " << BASE::QvTrajectoryStock_[i][k].transpose() << std::endl;
        std::cerr << "Qm: \n" << BASE::QmTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Rv: " << BASE::RvTrajectoryStock_[i][k].transpose() << std::endl;
        std::cerr << "Rm: \n" << BASE::RmTrajectoryStock_[i][k] << std::endl;
        std::cerr << "Pm: \n" << BASE::PmTrajectoryStock_[i][k] << std::endl;
        throw;
      }
    }
  }

  // Compute R inverse after inequalities are added to the cost
  // Compute it through the cholesky decomposition as we can reuse the factorization later on
  input_matrix_t RinvChol;
  LinearAlgebra::computeLinvTLinv(BASE::RmTrajectoryStock_[i][k], RinvChol);
  RmInverseTrajectoryStock_[i][k].noalias() = RinvChol * RinvChol.transpose();

  // constraint type 1 coefficients
  const auto nc1 = BASE::nc1TrajectoriesStock_[i][k];
  if (nc1 == 0) {
    DmDagerTrajectoryStock_[i][k].setZero();
    EvProjectedTrajectoryStock_[i][k].setZero();
    CmProjectedTrajectoryStock_[i][k].setZero();
    DmProjectedTrajectoryStock_[i][k].setZero();
    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    RmInvConstrainedCholTrajectoryStock_[i][k] = RinvChol;
  } else {
    dynamic_matrix_t Cm = BASE::CmTrajectoryStock_[i][k].topRows(nc1);
    dynamic_matrix_t Dm = BASE::DmTrajectoryStock_[i][k].topRows(nc1);

    // check numerical stability_
    if (BASE::ddpSettings_.checkNumericalStability_) {
      if (Dm.colPivHouseholderQr().rank() != nc1) {
        BASE::printString(
            ">>> WARNING: The state-input constraints are rank deficient "
            "(at time " +
            std::to_string(BASE::nominalTimeTrajectoriesStock_[i][k]) + ")!");
      }
    }

    // Constraint projectors are obtained at once
    dynamic_matrix_t DmDager, DdaggerT_R_Ddagger_Chol;
    ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol,
                                                     RmInvConstrainedCholTrajectoryStock_[i][k]);

    // Projected Constraints
    DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
    EvProjectedTrajectoryStock_[i][k].noalias() = DmDager * BASE::EvTrajectoryStock_[i][k].head(nc1);
    CmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Cm;
    DmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Dm;

    // Am constrained
    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    AmConstrainedTrajectoryStock_[i][k].noalias() -= BASE::BmTrajectoryStock_[i][k] * CmProjectedTrajectoryStock_[i][k];

    // Qm constrained
    state_matrix_t PmTransDmDagerCm = BASE::PmTrajectoryStock_[i][k].transpose() * CmProjectedTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
    dynamic_matrix_t Cm_RProjected_Cm_Chol = DdaggerT_R_Ddagger_Chol.transpose() * Cm;
    QmConstrainedTrajectoryStock_[i][k].noalias() += Cm_RProjected_Cm_Chol.transpose() * Cm_RProjected_Cm_Chol;

    // Qv constrained
    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k].noalias() -= CmProjectedTrajectoryStock_[i][k].transpose() * BASE::RvTrajectoryStock_[i][k];
  }

  // making sure that constrained Qm is PSD
  if (BASE::ddpSettings_.useMakePSD_) {
    LinearAlgebra::makePSD(QmConstrainedTrajectoryStock_[i][k]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateStateInputConstraintLagrangian(
		const scalar_t& time,
		const state_vector_t& state,
		dynamic_vector_t& nu) const {

	size_t activeSubsystem = BASE::findActivePartitionIndex(BASE::partitioningTimes_, time);

	state_vector_t xNominal;
	EigenLinearInterpolation<state_vector_t> xNominalFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &BASE::nominalStateTrajectoriesStock_[activeSubsystem]);
	auto greatestLessTimeStampIndex = xNominalFunc.interpolate(time, xNominal);

	state_input_matrix_t Bm;
	EigenLinearInterpolation<state_input_matrix_t> BmFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &BASE::BmTrajectoryStock_[activeSubsystem]);
	BmFunc.interpolate(time, Bm, greatestLessTimeStampIndex);

	input_state_matrix_t Pm;
	EigenLinearInterpolation<input_state_matrix_t> PmFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &BASE::PmTrajectoryStock_[activeSubsystem]);
	PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);

	input_vector_t Rv;
	EigenLinearInterpolation<input_vector_t> RvFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &BASE::RvTrajectoryStock_[activeSubsystem]);
	RvFunc.interpolate(time, Rv, greatestLessTimeStampIndex);

	input_matrix_t Rm;
	EigenLinearInterpolation<input_matrix_t> RmFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &BASE::RmTrajectoryStock_[activeSubsystem]);
	RmFunc.interpolate(time, Rm, greatestLessTimeStampIndex);

	input_vector_t EvProjected;
	EigenLinearInterpolation<input_vector_t> EvProjectedFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &EvProjectedTrajectoryStock_[activeSubsystem]);
	EvProjectedFunc.interpolate(time, EvProjected, greatestLessTimeStampIndex);

	input_state_matrix_t CmProjected;
	EigenLinearInterpolation<input_state_matrix_t> CmProjectedFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &CmProjectedTrajectoryStock_[activeSubsystem]);
	CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);

	input_constraint1_matrix_t DmDager;
	EigenLinearInterpolation<input_constraint1_matrix_t> DmDagerFunc(
			&BASE::nominalTimeTrajectoriesStock_[activeSubsystem], &DmDagerTrajectoryStock_[activeSubsystem]);
	DmDagerFunc.interpolate(time, DmDager, greatestLessTimeStampIndex);

	state_vector_t costate;
	BASE::getValueFunctionStateDerivative(time, xNominal, costate);

	if(greatestLessTimeStampIndex < 0){
		greatestLessTimeStampIndex = 0;
	}

	const size_t nc1 = BASE::nc1TrajectoriesStock_[activeSubsystem][greatestLessTimeStampIndex];
	state_vector_t deltaX = state - xNominal;
	deltaX.setZero();
	dynamic_input_matrix_t DmDagerTransRm = DmDager.leftCols(nc1).transpose() * Rm;

	nu = DmDagerTransRm * (CmProjected*deltaX+EvProjected) -
			DmDager.leftCols(nc1).transpose() * (Pm*deltaX + Bm.transpose()*costate+Rv);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::calculateController() {
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      BASE::nominalControllersStock_[i].clear();
      continue;
    }

    const size_t N = BASE::SsTimeTrajectoryStock_[i].size();

    BASE::nominalControllersStock_[i].timeStamp_ = BASE::SsTimeTrajectoryStock_[i];
    BASE::nominalControllersStock_[i].gainArray_.resize(N);
    BASE::nominalControllersStock_[i].biasArray_.resize(N);
    BASE::nominalControllersStock_[i].deltaBiasArray_.resize(N);

    // if the partition is not active
    if (N == 0) {
      continue;
    }

    // initialize interpolating function
    for (size_t j = 0; j < BASE::ddpSettings_.nThreads_; j++) {
      // functions for controller
      BASE::nominalStateFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::nominalStateTrajectoriesStock_[i]));
      BASE::nominalInputFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::nominalInputTrajectoriesStock_[i]));
      BmFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::BmTrajectoryStock_[i]));
      PmFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::PmTrajectoryStock_[i]));
      RmInverseFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(RmInverseTrajectoryStock_[i]));
      RvFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(BASE::RvTrajectoryStock_[i]));
      EvProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(EvProjectedTrajectoryStock_[i]));
      CmProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(CmProjectedTrajectoryStock_[i]));
      DmProjectedFunc_[j].setData(&(BASE::nominalTimeTrajectoriesStock_[i]), &(DmProjectedTrajectoryStock_[i]));
    }  // end of j loop

    // current partition update
    BASE::constraintStepSize_ = BASE::initialControllerDesignStock_[i] ? 0.0 : BASE::ddpSettings_.constraintStepSize_;

    /*
     * perform the calculatePartitionController for partition i
     */
    calculatePartitionController(i);

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;
  const auto time = BASE::SsTimeTrajectoryStock_[i][k];

  // local variables
  state_vector_t nominalState;
  input_vector_t nominalInput;
  state_input_matrix_t Bm;
  input_state_matrix_t Pm;
  input_vector_t Rv;
  input_matrix_t RmInverse;
  input_vector_t EvProjected;
  input_state_matrix_t CmProjected;
  input_matrix_t DmProjected;

  // interpolate
  const auto indexAlpha = BASE::nominalStateFunc_[workerIndex].interpolate(time, nominalState);
  BASE::nominalInputFunc_[workerIndex].interpolate(indexAlpha, nominalInput);
  BmFunc_[workerIndex].interpolate(indexAlpha, Bm);
  PmFunc_[workerIndex].interpolate(indexAlpha, Pm);
  RvFunc_[workerIndex].interpolate(indexAlpha, Rv);
  RmInverseFunc_[workerIndex].interpolate(indexAlpha, RmInverse);
  EvProjectedFunc_[workerIndex].interpolate(indexAlpha, EvProjected);
  CmProjectedFunc_[workerIndex].interpolate(indexAlpha, CmProjected);
  DmProjectedFunc_[workerIndex].interpolate(indexAlpha, DmProjected);

  // Lm
  Pm.noalias() += Bm.transpose() * BASE::SmTrajectoryStock_[i][k];  // Avoid temporary in the product
  input_state_matrix_t Lm = RmInverse * Pm;

  // Lv, Lve
  Rv.noalias() += Bm.transpose() * BASE::SvTrajectoryStock_[i][k];
  input_vector_t Lv = RmInverse * Rv;
  input_vector_t Lve = RmInverse * (Bm.transpose() * BASE::SveTrajectoryStock_[i][k]);

  input_matrix_t DmNullProjection = input_matrix_t::Identity() - DmProjected;

  // Feedback gains K
  BASE::nominalControllersStock_[i].gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k].noalias() -= DmNullProjection * Lm;

  // Bias input
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k] * nominalState -
                                                    BASE::constraintStepSize_ * (DmNullProjection * Lve + EvProjected);
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -DmNullProjection * Lv;

  // checking the numerical stability of the controller parameters
  if (BASE::ddpSettings_.checkNumericalStability_) {
    try {
      if (!BASE::nominalControllersStock_[i].gainArray_[k].allFinite()) {
        throw std::runtime_error("Feedback gains are unstable.");
      }
      if (!BASE::nominalControllersStock_[i].deltaBiasArray_[k].allFinite()) {
        throw std::runtime_error("feedForwardControl is unstable.");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].timeStamp_[k] << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::solveSlqRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex,
                                                                    const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                                                    const eigen_scalar_t& sFinal, const state_vector_t& SveFinal) {
  // solve Sm, Sv, s
  solveRiccatiEquationsWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);

  // Solve Sve
  solveErrorRiccatiEquationWorker(workerIndex, partitionIndex, SveFinal);

  // testing the numerical stability of the Riccati equations
  int N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sve is unstable");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("s is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "Sve[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:  \t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        exit(0);
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& SLQ_BASE<STATE_DIM, INPUT_DIM>::settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::setupOptimizer(size_t numPartitions) {
  BASE::setupOptimizer(numPartitions);

  // constraint type 1 coefficients
  DmDagerTrajectoryStock_.resize(numPartitions);
  AmConstrainedTrajectoryStock_.resize(numPartitions);
  QmConstrainedTrajectoryStock_.resize(numPartitions);
  QvConstrainedTrajectoryStock_.resize(numPartitions);
  EvProjectedTrajectoryStock_.resize(numPartitions);
  CmProjectedTrajectoryStock_.resize(numPartitions);
  DmProjectedTrajectoryStock_.resize(numPartitions);
  RmInvConstrainedCholTrajectoryStock_.resize(numPartitions);
  RmInverseTrajectoryStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::solveRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal,
                                                                 const state_vector_t& SvFinal, const eigen_scalar_t& sFinal) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &AmConstrainedTrajectoryStock_[partitionIndex],
      &BASE::BmTrajectoryStock_[partitionIndex], &BASE::qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &BASE::RvTrajectoryStock_[partitionIndex],
      &RmInvConstrainedCholTrajectoryStock_[partitionIndex], &BASE::PmTrajectoryStock_[partitionIndex],
      &BASE::nominalEventsPastTheEndIndecesStock_[partitionIndex], &BASE::qFinalStock_[partitionIndex],
      &BASE::QvFinalStock_[partitionIndex], &BASE::QmFinalStock_[partitionIndex]);

  // Const partition containers
  const auto& nominalTimeTrajectory = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  const auto& nominalEventsPastTheEndIndices = BASE::nominalEventsPastTheEndIndecesStock_[partitionIndex];

  // Modified partition containers
  auto& SsNormalizedTime = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex];
  auto& SsNormalizedEventsPastTheEndIndices = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex];
  auto& SsTimeTrajectory = BASE::SsTimeTrajectoryStock_[partitionIndex];
  auto& SmTrajectory = BASE::SmTrajectoryStock_[partitionIndex];
  auto& SvTrajectory = BASE::SvTrajectoryStock_[partitionIndex];
  auto& sTrajectory = BASE::sTrajectoryStock_[partitionIndex];

  // Convert final value of value function in vector format
  s_vector_t allSsFinal;
  riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // Clear output containers
  SsNormalizedTime.clear();
  SsNormalizedEventsPastTheEndIndices.clear();
  s_vector_array_t allSsTrajectory;

  /*
   *  The riccati equations are solved backwards in time
   *  the SsNormalized time is therefore filled with negative time in the reverse order, for example:
   *  nominalTime = [0.0, 1.0, 2.0, ..., 10.0]
   *  SsNormalized = [-10.0, ..., -2.0, -1.0, -0.0]
   *
   *  Depending on settings_.useNominalTimeForBackwardPass_:
   *  if true: the integration will produce the same time nodes set in nominalTime (=resulting from the forward pass),
   *  if false: the SsNormalized time is a result of adaptive integration.
   */
  if (settings_.useNominalTimeForBackwardPass_) {
    integrateRiccatiEquationNominalTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex],
                                        nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                        SsNormalizedEventsPastTheEndIndices, allSsTrajectory);
  } else {
    integrateRiccatiEquationAdaptiveTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex],
                                         nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                         SsNormalizedEventsPastTheEndIndices, allSsTrajectory);
  }

  // De-normalize time and convert value function to matrix format
  size_t outputN = SsNormalizedTime.size();
  SsTimeTrajectory.resize(outputN);
  SmTrajectory.resize(outputN);
  SvTrajectory.resize(outputN);
  sTrajectory.resize(outputN);
  for (size_t k = 0; k < outputN; k++) {
    SsTimeTrajectory[k] = -SsNormalizedTime[outputN - 1 - k];
    riccati_equations_t::convert2Matrix(allSsTrajectory[outputN - 1 - k], SmTrajectory[k], SvTrajectory[k], sTrajectory[k]);
  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationNominalTime(
    IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator, riccati_equations_t& riccatiEquation,
    const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
    scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices, s_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const size_t nominalTimeSize = nominalTimeTrajectory.size();
  const size_t numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Normalize time
  SsNormalizedTime.resize(nominalTimeSize);
  for (size_t k = 0; k < nominalTimeSize; k++) {
    SsNormalizedTime[nominalTimeSize - 1 - k] = -nominalTimeTrajectory[k];
  }

  // Normalized switching time indices, start and end of the partition are added at the beginning and end
  size_array_t SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (int k = static_cast<int>(numEvents) - 1; k >= 0; k--) {
    size_t index = nominalEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize - index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize);

  // integrating the Riccati equations
  SsNormalizedEventsPastTheEndIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (size_t i = 0; i <= numEvents; i++) {
    typename scalar_array_t::const_iterator beginTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i];
    typename scalar_array_t::const_iterator endTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      riccatiIntegrator.integrate(allSsFinal, beginTimeItr, endTimeItr, allSsTrajectory, BASE::ddpSettings_.minTimeStep_,
                                  BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      allSsTrajectory.push_back(allSsFinal);
    }

    if (i < numEvents) {
      SsNormalizedEventsPastTheEndIndices.push_back(allSsTrajectory.size());
      riccatiEquation.computeJumpMap(*endTimeItr, allSsTrajectory.back(), allSsFinal);
    }
  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != nominalTimeSize) {
    throw std::runtime_error("allSsTrajectory size is incorrect.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationAdaptiveTime(
    IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator, riccati_equations_t& riccatiEquation,
    const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
    scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices, s_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const size_t nominalTimeSize = nominalTimeTrajectory.size();
  const size_t numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Extract switching times from eventIndices and normalize them
  scalar_array_t SsNormalizedSwitchingTimes;
  SsNormalizedSwitchingTimes.reserve(numEvents + 2);
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.back());
  for (int k = static_cast<int>(numEvents) - 1; k >= 0; k--) {
    size_t index = nominalEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory[index]);
  }
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.front());

  // integrating the Riccati equations
  SsNormalizedTime.reserve(maxNumSteps);
  SsNormalizedEventsPastTheEndIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (size_t i = 0; i <= numEvents; i++) {
    scalar_t beginTime = SsNormalizedSwitchingTimes[i];
    scalar_t endTime = SsNormalizedSwitchingTimes[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (beginTime < endTime) {
      riccatiIntegrator.integrate(allSsFinal, beginTime, endTime, allSsTrajectory, SsNormalizedTime, BASE::ddpSettings_.minTimeStep_,
                                  BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      SsNormalizedTime.push_back(endTime);
      allSsTrajectory.push_back(allSsFinal);
    }

    // if not the last interval which definitely does not have any event at
    // its final time (there is no even at the beginning of partition)
    if (i < numEvents) {
      SsNormalizedEventsPastTheEndIndices.push_back(allSsTrajectory.size());
      riccatiEquation.computeJumpMap(endTime, allSsTrajectory.back(), allSsFinal);
    }
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_BASE<STATE_DIM, INPUT_DIM>::solveErrorRiccatiEquationWorker(size_t workerIndex, size_t partitionIndex,
                                                                     const state_vector_t& SveFinal) {
  // Const partition containers
  const auto& nominalTimeTrajectory = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  const auto& SsNormalizedTime = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex];
  const auto& SsNormalizedEventsPastTheEndIndices = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex];
  const auto& SsTimeTrajectory = BASE::SsTimeTrajectoryStock_[partitionIndex];
  const auto& SmTrajectory = BASE::SmTrajectoryStock_[partitionIndex];

  // const Data
  const auto& PmTrajectory = BASE::PmTrajectoryStock_[partitionIndex];
  const auto& BmTrajectory = BASE::BmTrajectoryStock_[partitionIndex];
  const auto& AmTrajectory = AmConstrainedTrajectoryStock_[partitionIndex];
  const auto& RmInvCholTrajectory = RmInvConstrainedCholTrajectoryStock_[partitionIndex];
  const auto& RmTrajectory = BASE::RmTrajectoryStock_[partitionIndex];
  const auto& RmInvTrajectory = RmInverseTrajectoryStock_[partitionIndex];
  const auto& EvTrajectory = EvProjectedTrajectoryStock_[partitionIndex];
  const auto& CmTrajectory = CmProjectedTrajectoryStock_[partitionIndex];

  // Modified partition containers
  auto& SveTrajectory = BASE::SveTrajectoryStock_[partitionIndex];

  /*
   * Type_1 constraints error correction compensation
   */
  const size_t nominalTimeSize = nominalTimeTrajectory.size();
  const size_t ssTimeSize = SsNormalizedTime.size();
  const size_t numEvents = SsNormalizedEventsPastTheEndIndices.size();

  // Skip calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation
  if (BASE::ddpSettings_.simulationIsConstrained_) {
    SveTrajectory.resize(ssTimeSize, state_vector_t::Zero());
    return;
  }

  /*
   * Calculating the coefficients of the error equation
   */
  SmFuncs_[workerIndex].setData(&SsTimeTrajectory, &SmTrajectory);
  state_vector_array_t GvTrajectory(nominalTimeSize);
  state_matrix_array_t GmTrajectory(nominalTimeSize);
  state_matrix_t Sm;
  input_state_matrix_t Lm;
  input_vector_t RmEv;
  for (int k = static_cast<int>(nominalTimeSize) - 1; k >= 0; k--) {
    // Sm
    SmFuncs_[workerIndex].interpolate(nominalTimeTrajectory[k], Sm);
    // Lm
    Lm = PmTrajectory[k];
    Lm.noalias() += BmTrajectory[k].transpose() * Sm;

    GmTrajectory[k] = AmTrajectory[k];
    GmTrajectory[k].noalias() -= (BmTrajectory[k] * RmInvCholTrajectory[k]) * (RmInvCholTrajectory[k].transpose() * Lm);

    RmEv.noalias() = RmTrajectory[k] * EvTrajectory[k];
    GvTrajectory[k].noalias() = CmTrajectory[k].transpose() * RmEv;
    GvTrajectory[k].noalias() -= Lm.transpose() * (RmInvTrajectory[k].transpose() * RmEv);
  }  // end of k loop

  // set data for error equations
  errorEquationPtrStock_[workerIndex]->resetNumFunctionCalls();
  errorEquationPtrStock_[workerIndex]->setData(&nominalTimeTrajectory, &GvTrajectory, &GmTrajectory);

  // max number of steps of integration
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // clear output
  SveTrajectory.clear();
  SveTrajectory.reserve(maxNumSteps);

  // final value from the previous partition Riccati equations
  state_vector_t SveFinalInternal = SveFinal;

  // normalized switching times
  size_array_t SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (size_t k = 0; k < numEvents; k++) {
    const auto index = SsNormalizedEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimesIndices.push_back(index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(ssTimeSize);

  // integrating the Error Riccati equation
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= numEvents; i++) {
    beginTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      errorIntegratorPtrStock_[workerIndex]->integrate(SveFinalInternal, beginTimeItr, endTimeItr, SveTrajectory,
                                                       BASE::ddpSettings_.minTimeStep_, BASE::ddpSettings_.absTolODE_,
                                                       BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      SveTrajectory.push_back(SveFinalInternal);
    }

    if (i < numEvents) {
      errorEquationPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, SveTrajectory.back(), SveFinalInternal);
    }

  }  // end of i loop

  // check size
  if (SveTrajectory.size() != ssTimeSize) {
    throw std::runtime_error("SveTrajectory size is incorrect.");
  }

  // Reverse Sve to match normal time direction
  std::reverse(SveTrajectory.begin(), SveTrajectory.end());
}

}  // namespace ocs2
