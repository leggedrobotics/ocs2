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

/*
 *  !! Copy of SLQ with the unfinished Hamiltonian implementation for the backward pass !!
 */

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SLQ_Hamiltonian(
    const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
    const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
    const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings /*= SLQ_Settings()*/,
    const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/, const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           settings.rolloutSettings_, logicRulesPtr, heuristicsFunctionPtr, "SLQ"),
      settings_(settings) {
  hamiltonianEquationPtrStock_.clear();
  hamiltonianEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  hamiltonianIntegratorPtrStock_.clear();
  hamiltonianIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  hamiltonianIncrementEquationPtrStock_.clear();
  hamiltonianIncrementEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  hamiltonianIncrementIntegratorPtrStock_.clear();
  hamiltonianIncrementIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    hamiltonianEquationPtrStock_.emplace_back(new hamiltonian_equation_t());
    hamiltonianIntegratorPtrStock_.emplace_back(new ODE45<hamiltonian_equation_t::LTI_DIM_>(hamiltonianEquationPtrStock_[i]));

    hamiltonianIncrementEquationPtrStock_.emplace_back(new hamiltonian_increment_equation_t());
    hamiltonianIncrementIntegratorPtrStock_.emplace_back(
        new ODE45<hamiltonian_increment_equation_t::LTI_DIM_>(hamiltonianIncrementEquationPtrStock_[i]));
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateOptimalControlProblem() {
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
    if (BASE::ddpSettings_.useRiccatiSolver_) {
      RmInvConstrainedCholTrajectoryStock_[i].resize(N);
    } else {
      BmConstrainedTrajectoryStock_[i].resize(N);
      PmConstrainedTrajectoryStock_[i].resize(N);
      RvConstrainedTrajectoryStock_[i].resize(N);
    }
  }  // end of i loop

  // base method should be called after the resizes
  BASE::approximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateLQWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                               const size_t& timeIndex) {
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateConstrainedLQWorker(size_t workerIndex, const size_t& i,
                                                                                          const size_t& k,
                                                                                          const scalar_t& stateConstraintPenalty) {
  // constraint type 2 coefficients
  const size_t& nc2 = BASE::nc2TrajectoriesStock_[i][k];
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
        if (LinearAlgebra::eigenvalues(BASE::QmTrajectoryStock_[i][k]).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(BASE::QmTrajectoryStock_[i][k]).real().minCoeff()) + ".");
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
        if (LinearAlgebra::eigenvalues(BASE::RmTrajectoryStock_[i][k]).real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("R matrix is not positive definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(BASE::RmTrajectoryStock_[i][k]).real().minCoeff()) + ".");
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
  RmInverseTrajectoryStock_[i][k] = RinvChol * RinvChol.transpose();

  // constraint type 1 coefficients
  const size_t& nc1 = BASE::nc1TrajectoriesStock_[i][k];
  if (nc1 == 0) {
    DmDagerTrajectoryStock_[i][k].setZero();
    EvProjectedTrajectoryStock_[i][k].setZero();
    CmProjectedTrajectoryStock_[i][k].setZero();
    DmProjectedTrajectoryStock_[i][k].setZero();

    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    if (BASE::ddpSettings_.useRiccatiSolver_) {
      RmInvConstrainedCholTrajectoryStock_[i][k] = RinvChol;
    } else {
      BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k];
      PmConstrainedTrajectoryStock_[i][k] = BASE::PmTrajectoryStock_[i][k];
      RvConstrainedTrajectoryStock_[i][k] = BASE::RvTrajectoryStock_[i][k];
    }

  } else {
    dynamic_matrix_t Cm = BASE::CmTrajectoryStock_[i][k].topRows(nc1);
    dynamic_matrix_t Dm = BASE::DmTrajectoryStock_[i][k].topRows(nc1);

    // check numerical stability_
    if (BASE::ddpSettings_.checkNumericalStability_ && nc1 > 0) {
      if (LinearAlgebra::rank(Dm) != nc1) {
        BASE::printString(
            ">>> WARNING: The state-input constraints are rank deficient "
            "(at time " +
            std::to_string(BASE::nominalTimeTrajectoriesStock_[i][k]) + ")!");
      }
    }

    // Constraint projectors are obtained at once
    dynamic_matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
    ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

    DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
    EvProjectedTrajectoryStock_[i][k].noalias() = DmDager * BASE::EvTrajectoryStock_[i][k].head(nc1);
    CmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Cm;
    DmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Dm;

    AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
    AmConstrainedTrajectoryStock_[i][k].noalias() -= BASE::BmTrajectoryStock_[i][k] * CmProjectedTrajectoryStock_[i][k];

    state_matrix_t PmTransDmDagerCm = BASE::PmTrajectoryStock_[i][k].transpose() * CmProjectedTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
    dynamic_matrix_t Cm_RProjected_Cm_Chol = DdaggerT_R_Ddagger_Chol.transpose() * Cm;
    QmConstrainedTrajectoryStock_[i][k].noalias() += Cm_RProjected_Cm_Chol.transpose() * Cm_RProjected_Cm_Chol;

    QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
    QvConstrainedTrajectoryStock_[i][k].noalias() -= CmProjectedTrajectoryStock_[i][k].transpose() * BASE::RvTrajectoryStock_[i][k];

    if (BASE::ddpSettings_.useRiccatiSolver_) {
      RmInvConstrainedCholTrajectoryStock_[i][k] = RinvConstrainedChol;
    } else {
      input_matrix_t DmNullSpaceProjection = input_matrix_t::Identity() - DmProjectedTrajectoryStock_[i][k];
      BmConstrainedTrajectoryStock_[i][k].noalias() = BASE::BmTrajectoryStock_[i][k] * DmNullSpaceProjection;
      PmConstrainedTrajectoryStock_[i][k].noalias() = DmNullSpaceProjection.transpose() * BASE::PmTrajectoryStock_[i][k];
      RvConstrainedTrajectoryStock_[i][k].noalias() = DmNullSpaceProjection.transpose() * BASE::RvTrajectoryStock_[i][k];
    }
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
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                     const size_t& timeIndex) {
  const size_t& i = partitionIndex;
  const size_t& k = timeIndex;
  const scalar_t& time = BASE::SsTimeTrajectoryStock_[i][k];

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

  input_state_matrix_t Lm = RmInverse * (Pm + Bm.transpose() * BASE::SmTrajectoryStock_[i][k]);
  input_vector_t Lv = RmInverse * (Rv + Bm.transpose() * BASE::SvTrajectoryStock_[i][k]);
  input_vector_t Lve = RmInverse * (Bm.transpose() * BASE::SveTrajectoryStock_[i][k]);

  input_matrix_t DmNullProjection = input_matrix_t::Identity() - DmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k] = -DmNullProjection * Lm - CmProjected;
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                       const state_matrix_t& SmFinal,
                                                                                       const state_vector_t& SvFinal,
                                                                                       const scalar_t sFinal) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &AmConstrainedTrajectoryStock_[partitionIndex],
      &BASE::BmTrajectoryStock_[partitionIndex], &BASE::qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &BASE::RvTrajectoryStock_[partitionIndex],
      &RmInvConstrainedCholTrajectoryStock_[partitionIndex], &BASE::PmTrajectoryStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::qFinalStock_[partitionIndex], &BASE::QvFinalStock_[partitionIndex],
      &BASE::QmFinalStock_[partitionIndex]);

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // Normalized start and final time and index for Riccati equation
  scalar_t finalNormalizedTime = (BASE::partitioningTimes_[partitionIndex] - scalingFinal) / scalingFactor;  // which is 1.0
  if (partitionIndex == BASE::initActivePartition_) {
    finalNormalizedTime = (BASE::initTime_ - scalingFinal) / scalingFactor;
  }
  scalar_t startNormalizedTime = (BASE::partitioningTimes_[partitionIndex + 1] - scalingFinal) / scalingFactor;  // which is 0.0
  if (partitionIndex == BASE::finalActivePartition_) {
    startNormalizedTime = (BASE::finalTime_ - scalingFinal) / scalingFactor;
  }

  // max number of steps of integration
  size_t maxNumSteps = BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, finalNormalizedTime - startNormalizedTime);

  // clear output containers
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].clear();
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].reserve(maxNumSteps);
  typename riccati_equations_t::s_vector_array_t allSsTrajectory(0);
  allSsTrajectory.reserve(maxNumSteps);
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].clear();
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].reserve(NE);

  // final value from the previous partition Riccati equations
  typename riccati_equations_t::s_vector_t allSsFinal;
  riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // normalized switching times
  scalar_array_t SsNormalizedSwitchingTimes;
  SsNormalizedSwitchingTimes.reserve(NE + 2);
  SsNormalizedSwitchingTimes.push_back(startNormalizedTime);
  for (int k = NE - 1; k >= 0; k--) {
    const size_t& index = BASE::nominalPostEventIndicesStock_[partitionIndex][k];
    const scalar_t& si = BASE::nominalTimeTrajectoriesStock_[partitionIndex][index];
    SsNormalizedSwitchingTimes.push_back((si - scalingFinal) / scalingFactor);
  }
  SsNormalizedSwitchingTimes.push_back(finalNormalizedTime);

  // integrating the Riccati equations
  for (size_t i = 0; i <= NE; i++) {
    const scalar_t& beginTime = SsNormalizedSwitchingTimes[i];
    const scalar_t& endTime = SsNormalizedSwitchingTimes[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (beginTime < endTime) {
      riccatiIntegratorPtrStock_[workerIndex]->integrate(
          allSsFinal, beginTime, endTime, allSsTrajectory, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex],
          BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].push_back(endTime);
      allSsTrajectory.push_back(allSsFinal);
    }

    // if not the last interval which definitely does not have any event at
    // its final time (there is no even at the beginning of partition)
    if (i < NE) {
      BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].push_back(allSsTrajectory.size());
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(endTime, allSsTrajectory.back(), allSsFinal);
    }

  }  // end of i loop

  // denormalizing time and constructing 'Sm', 'Sv', and 's'
  size_t NS = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
  BASE::SsTimeTrajectoryStock_[partitionIndex].resize(NS);
  BASE::SmTrajectoryStock_[partitionIndex].resize(NS);
  BASE::SvTrajectoryStock_[partitionIndex].resize(NS);
  BASE::sTrajectoryStock_[partitionIndex].resize(NS);
  for (size_t k = 0; k < NS; k++) {
    riccati_equations_t::convert2Matrix(allSsTrajectory[NS - 1 - k], BASE::SmTrajectoryStock_[partitionIndex][k],
                                        BASE::SvTrajectoryStock_[partitionIndex][k], BASE::sTrajectoryStock_[partitionIndex][k]);
    BASE::SsTimeTrajectoryStock_[partitionIndex][k] =
        scalingFactor * BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][NS - 1 - k] + scalingFinal;
  }  // end of k loop

  // testing the numerical stability of the Riccati equations
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = NS - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff() <
            -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error(
              "Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
              std::to_string(LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("s is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= NS) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsForNominalTimeWorker(
    size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const scalar_t sFinal) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &AmConstrainedTrajectoryStock_[partitionIndex],
      &BASE::BmTrajectoryStock_[partitionIndex], &BASE::qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &BASE::RvTrajectoryStock_[partitionIndex],
      &RmInvConstrainedCholTrajectoryStock_[partitionIndex], &BASE::PmTrajectoryStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::qFinalStock_[partitionIndex], &BASE::QvFinalStock_[partitionIndex],
      &BASE::QmFinalStock_[partitionIndex]);

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // normalized time
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][N - 1 - k] =
        (BASE::nominalTimeTrajectoriesStock_[partitionIndex][k] - scalingFinal) / scalingFactor;
  }

  // normalized event past the index
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
  for (size_t k = 0; k < NE; k++) {
    BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE - 1 - k] =
        N - BASE::nominalPostEventIndicesStock_[partitionIndex][k];
  }

  // max number of steps of integration
  const size_t maxNumSteps =
      BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].back() -
                                                                   BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

  // output containers resizing
  typename riccati_equations_t::s_vector_array_t allSsTrajectory(0);
  allSsTrajectory.reserve(maxNumSteps);

  // final value for the last Riccati equations plus final cost
  typename riccati_equations_t::s_vector_t allSsFinal;
  riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

  // normalized switching times
  size_array_t SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (int k = NE - 1; k >= 0; k--) {
    const size_t& index = BASE::nominalPostEventIndicesStock_[partitionIndex][k];
    SsNormalizedSwitchingTimesIndices.push_back(N - index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(N);

  // integrating the Riccati equations
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= NE; i++) {
    beginTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      riccatiIntegratorPtrStock_[workerIndex]->integrate(allSsFinal, beginTimeItr, endTimeItr, allSsTrajectory,
                                                         BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_,
                                                         BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      allSsTrajectory.push_back(allSsFinal);
    }

    if (i < NE) {
      riccatiEquationsPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, allSsTrajectory.back(), allSsFinal);
    }

  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != N) {
    throw std::runtime_error("allSsTrajectory size is incorrect.");
  }

  // denormalizing time and constructing 'Sm', 'Sv', and 's'
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  BASE::sTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    riccati_equations_t::convert2Matrix(allSsTrajectory[N - 1 - k], BASE::SmTrajectoryStock_[partitionIndex][k],
                                        BASE::SvTrajectoryStock_[partitionIndex][k], BASE::sTrajectoryStock_[partitionIndex][k]);
  }  // end of k loop

  // testing the numerical stability of the Riccati equations
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff() <
            -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error(
              "Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
              std::to_string(LinearAlgebra::eigenvalues(BASE::SmTrajectoryStock_[partitionIndex][k]).real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sv is unstable.");
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
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveErrorRiccatiEquationWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                           const state_vector_t& SveFinal) {
  /*
   * Type_1 constraints error correction compensation
   */

  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NS = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
  const size_t NE = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // Skip calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation
  if (BASE::ddpSettings_.simulationIsConstrained_) {
    BASE::SveTrajectoryStock_[partitionIndex].resize(NS);
    for (size_t k = 0; k < NS; k++) {
      BASE::SveTrajectoryStock_[partitionIndex][k].setZero();
    }
    return;
  }

  /*
   * Calculating the coefficients of the error equation
   */
  SmFuncs_[workerIndex].setData(&(BASE::SsTimeTrajectoryStock_[partitionIndex]), &(BASE::SmTrajectoryStock_[partitionIndex]));
  state_vector_array_t GvTrajectory(N);
  state_matrix_array_t GmTrajectory(N);
  state_matrix_t Sm;
  input_state_matrix_t Lm;
  input_vector_t RmEv;
  for (int k = N - 1; k >= 0; k--) {
    // Sm
    SmFuncs_[workerIndex].interpolate(BASE::nominalTimeTrajectoriesStock_[partitionIndex][k], Sm);
    // Lm
    Lm = BASE::PmTrajectoryStock_[partitionIndex][k];
    Lm.noalias() += BASE::BmTrajectoryStock_[partitionIndex][k].transpose() * Sm;

    GmTrajectory[k] = AmConstrainedTrajectoryStock_[partitionIndex][k];
    GmTrajectory[k].noalias() -= (BASE::BmTrajectoryStock_[partitionIndex][k] * RmInvConstrainedCholTrajectoryStock_[partitionIndex][k]) *
                                 (RmInvConstrainedCholTrajectoryStock_[partitionIndex][k].transpose() * Lm);

    RmEv.noalias() = BASE::RmTrajectoryStock_[partitionIndex][k] * EvProjectedTrajectoryStock_[partitionIndex][k];
    GvTrajectory[k] = CmProjectedTrajectoryStock_[partitionIndex][k].transpose() * RmEv;
    GvTrajectory[k].noalias() -= Lm.transpose() * (RmInverseTrajectoryStock_[partitionIndex][k].transpose() * RmEv);
  }  // end of k loop

  // set data for error equations
  errorEquationPtrStock_[workerIndex]->resetNumFunctionCalls();
  errorEquationPtrStock_[workerIndex]->setData(BASE::partitioningTimes_[partitionIndex], BASE::partitioningTimes_[partitionIndex + 1],
                                               &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &GvTrajectory, &GmTrajectory);

  // max number of steps of integration
  const size_t maxNumSteps =
      BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].back() -
                                                                   BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

  // clear output
  state_vector_array_t SveTrajectory(0);
  SveTrajectory.reserve(maxNumSteps);

  // final value from the previous partition Riccati equations
  state_vector_t SveFinalInternal = SveFinal;

  // normalized switching times
  size_array_t SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (size_t k = 0; k < NE; k++) {
    const size_t& index = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][k];
    SsNormalizedSwitchingTimesIndices.push_back(index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(NS);

  // integrating the Error Riccati equation
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= NE; i++) {
    beginTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    // solve Riccati equations if interval length is not zero (no event time at final time)
    if (*beginTimeItr < *(endTimeItr - 1)) {
      errorIntegratorPtrStock_[workerIndex]->integrate(SveFinalInternal, beginTimeItr, endTimeItr, SveTrajectory,
                                                       BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_,
                                                       BASE::ddpSettings_.relTolODE_, maxNumSteps, true);
    } else {
      SveTrajectory.push_back(SveFinalInternal);
    }

    if (i < NE) {
      errorEquationPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, SveTrajectory.back(), SveFinalInternal);
    }

  }  // end of i loop

  // check size
  if (SveTrajectory.size() != NS) {
    throw std::runtime_error("SveTrajectory size is incorrect.");
  }

  // constructing Sve
  BASE::SveTrajectoryStock_[partitionIndex].resize(SveTrajectory.size());
  std::reverse_copy(SveTrajectory.begin(), SveTrajectory.end(), BASE::SveTrajectoryStock_[partitionIndex].begin());

  // Testing the numerical stability
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (size_t k = 0; k < NS; k++) {
      // testing the numerical stability of the Riccati error equation
      try {
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sve is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < NS; kp++) {
          std::cerr << "Sve[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        for (int kp = 0; kp + 1 < BASE::nominalTimeTrajectoriesStock_[partitionIndex].size(); kp++) {
          std::cerr << "Gm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t" << GmTrajectory[kp].transpose().norm()
                    << std::endl;
          std::cerr << "Gv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t" << GvTrajectory[kp].transpose().norm()
                    << std::endl;
        }
        throw;
      }
    }  // end of k loop
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSlqRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex,
                                                                                          const state_matrix_t& SmFinal,
                                                                                          const state_vector_t& SvFinal,
                                                                                          const scalar_t sFinal,
                                                                                          const state_vector_t& SveFinal) {
  if (settings_.useNominalTimeForBackwardPass_) {
    solveRiccatiEquationsForNominalTimeWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);
  } else {
    solveRiccatiEquationsWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);
  }

  solveErrorRiccatiEquationWorker(workerIndex, partitionIndex, SveFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
template <int DIM1, int DIM2>
Eigen::Matrix<typename SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t, DIM1, DIM2>
SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveLTI(
    const std::shared_ptr<IntegratorBase<DIM1 * DIM2>>& firstOrderOdeIntegratorPtr, const Eigen::Matrix<scalar_t, DIM1, DIM2>& x0,
    const scalar_t& deltaTime) {
  // dx = A x + B u
  using lti_equation_t = LTI_Equations<DIM1, DIM2, scalar_t>;
  using vectorized_state_t = typename lti_equation_t::vectorized_state_t;
  using vectorized_state_array_t = typename lti_equation_t::vectorized_state_array_t;

  scalar_array_t timeTrajectory{0.0, deltaTime};
  vectorized_state_array_t stateTrajectory;
  stateTrajectory.reserve(2);

#if DIM2 == 1
  firstOrderOdeIntegrator.integrate(x0, timeTrajectory.begin(), timeTrajectory.end(), stateTrajectory, BASE::ddpSettings_.timeStep_,
                                    BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_);

  return stateTrajectory.back();
#else
  vectorized_state_t x0Vectorized;
  lti_equation_t::convert2Vector(x0, x0Vectorized);
  firstOrderOdeIntegratorPtr->integrate(x0Vectorized, timeTrajectory.begin(), timeTrajectory.end(), stateTrajectory,
                                        BASE::ddpSettings_.timeStep_, BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_);

  Eigen::Matrix<scalar_t, DIM1, DIM2> xf;
  lti_equation_t::convert2Matrix(stateTrajectory.back(), xf);

  return xf;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
Eigen::Matrix<typename SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t, 2 * STATE_DIM, STATE_DIM>
SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::integrateHamiltonian(size_t workerIndex,
                                                                           const Eigen::Matrix<scalar_t, 2 * STATE_DIM, 2 * STATE_DIM>& Hm,
                                                                           const Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM>& x0,
                                                                           const scalar_t& deltaTime) {
  const bool useExpMethod = false;
  if (useExpMethod) {
    //
    //		Eigen::HessenbergDecomposition<Eigen::MatrixXd> hessA(DIM1);
    //		hessA.compute(A);
    //
    //		Eigen::Matrix<double, DIM1, DIM2> xf = hessA.matrixQ()
    //							* (hessA.matrixH()*deltaTime + Eigen::MatrixXd::Identity(DIM1,
    // DIM1)*1e-3*deltaTime).exp()
    //							* hessA.matrixQ().transpose() * x0;

    //		Eigen::EigenSolver<Eigen::MatrixXd> esA(DIM1);
    //		esA.compute(A*deltaTime, /* computeEigenvectors = */ true);
    //		Eigen::VectorXcd expLamda = esA.eigenvalues().array().exp();
    //		Eigen::MatrixXcd V = esA.eigenvectors();
    //
    //		Eigen::Matrix<double, DIM1, DIM2> xf = (V * expLamda.asDiagonal() * V.inverse()).real() * x0;

    Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> xf = (Hm * deltaTime).exp() * x0;
    return xf;

  } else {
    // set data
    static Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> GvZero = Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM>::Zero();
    hamiltonianEquationPtrStock_[workerIndex]->setData(&Hm, &GvZero);

    return solveLTI<2 * STATE_DIM, STATE_DIM>(hamiltonianIntegratorPtrStock_[workerIndex], x0, deltaTime);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
Eigen::Matrix<typename SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t, STATE_DIM, 1>
SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::integrateIncrement(size_t workerIndex,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>& Gm,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, 1>& Gv,
                                                                         const Eigen::Matrix<scalar_t, STATE_DIM, 1>& x0,
                                                                         const scalar_t& deltaTime) {
  // set data
  hamiltonianIncrementEquationPtrStock_[workerIndex]->setData(&Gm, &Gv);

  return solveLTI<STATE_DIM, 1>(hamiltonianIncrementIntegratorPtrStock_[workerIndex], x0, deltaTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::fullRiccatiBackwardSweepWorker(
    size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
    const state_vector_t& SveFinal, const scalar_t sFinal, const scalar_t& constraintStepSize) {
  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

  // Riccati parameters
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  BASE::SveTrajectoryStock_[partitionIndex].resize(N);
  BASE::sTrajectoryStock_[partitionIndex].resize(N);

  // controller parameters
  BASE::nominalControllersStock_[partitionIndex].timeStamp_ = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  BASE::nominalControllersStock_[partitionIndex].gainArray_.resize(N);
  BASE::nominalControllersStock_[partitionIndex].biasArray_.resize(N);
  BASE::nominalControllersStock_[partitionIndex].deltaBiasArray_.resize(N);

  state_matrix_t _Gm;
  state_vector_t _Gv, _Gve;
  input_state_matrix_t _Lm, _LmConstrained;
  input_vector_t _LvConstrained, _LveConstrained;
  Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> _X_H_0, _X_H_1;
  Eigen::Matrix<scalar_t, 2 * STATE_DIM, 2 * STATE_DIM> _H;
  _X_H_0.template topRows<STATE_DIM>().setIdentity();

  BASE::SmTrajectoryStock_[partitionIndex][N - 1] = SmFinal;
  BASE::SvTrajectoryStock_[partitionIndex][N - 1] = SvFinal;
  BASE::SveTrajectoryStock_[partitionIndex][N - 1] = SveFinal;
  BASE::sTrajectoryStock_[partitionIndex][N - 1] = sFinal;

  LmFunc_(partitionIndex, N - 1, _Lm);
  LmConstrainedFunc_(partitionIndex, N - 1, _Lm, _LmConstrained);
  LvConstrainedFunc_(partitionIndex, N - 1, _LvConstrained);
  LveConstrainedFunc_(partitionIndex, N - 1, _LveConstrained);

  ControllerFunc_(partitionIndex, N - 1, constraintStepSize, _LmConstrained, _LvConstrained, _LveConstrained);

  int remainingEvents = BASE::nominalPostEventIndicesStock_[partitionIndex].size();
  bool eventDetected = false;
  scalar_t deltaT = 0.0;
  for (int k = N - 2; k >= 0; k--) {
    // check if an event is detected.
    if (remainingEvents > 0 && k + 1 == BASE::nominalPostEventIndicesStock_[partitionIndex][remainingEvents - 1]) {
      eventDetected = true;
      remainingEvents--;
    } else {
      eventDetected = false;
    }

    deltaT = BASE::nominalTimeTrajectoriesStock_[partitionIndex][k] - BASE::nominalTimeTrajectoriesStock_[partitionIndex][k + 1];

    if (eventDetected) {
      //			if (BASE::ddpSettings_.useMakePSD_==true)
      // BASE::makePSD(BASE::QmFinalStock_[partitionIndex][remainingEvents]);
      BASE::SmTrajectoryStock_[partitionIndex][k] =
          BASE::SmTrajectoryStock_[partitionIndex][k + 1] + BASE::QmFinalStock_[partitionIndex][remainingEvents];

    } else {
      //			if (BASE::ddpSettings_.useMakePSD_==true)  BASE::makePSD(QmConstrainedTrajectoryStock_[partitionIndex][k]);
      _H.template topLeftCorner<STATE_DIM, STATE_DIM>() =
          AmConstrainedTrajectoryStock_[partitionIndex][k] - BmConstrainedTrajectoryStock_[partitionIndex][k] *
                                                                 RmInverseTrajectoryStock_[partitionIndex][k] *
                                                                 PmConstrainedTrajectoryStock_[partitionIndex][k];
      _H.template topRightCorner<STATE_DIM, STATE_DIM>() = 0.5 * BmConstrainedTrajectoryStock_[partitionIndex][k] *
                                                           RmInverseTrajectoryStock_[partitionIndex][k] *
                                                           BmConstrainedTrajectoryStock_[partitionIndex][k].transpose();
      _H.template bottomLeftCorner<STATE_DIM, STATE_DIM>() =
          2.0 * (QmConstrainedTrajectoryStock_[partitionIndex][k] - PmConstrainedTrajectoryStock_[partitionIndex][k].transpose() *
                                                                        RmInverseTrajectoryStock_[partitionIndex][k] *
                                                                        PmConstrainedTrajectoryStock_[partitionIndex][k]);
      _H.template bottomRightCorner<STATE_DIM, STATE_DIM>() = -_H.template topLeftCorner<STATE_DIM, STATE_DIM>().transpose();

      _X_H_0.template bottomRows<STATE_DIM>() = -2.0 * BASE::SmTrajectoryStock_[partitionIndex][k + 1];
      _X_H_1 = integrateHamiltonian(workerIndex, _H, _X_H_0, deltaT);
      BASE::SmTrajectoryStock_[partitionIndex][k] =
          -0.5 * _X_H_1.template bottomRows<STATE_DIM>() * _X_H_1.template topRows<STATE_DIM>().inverse();
    }

    LmFunc_(partitionIndex, k, _Lm);
    LmConstrainedFunc_(partitionIndex, k, _Lm, _LmConstrained);

    if (eventDetected) {
      BASE::SvTrajectoryStock_[partitionIndex][k] =
          BASE::SvTrajectoryStock_[partitionIndex][k + 1] + BASE::QvFinalStock_[partitionIndex][remainingEvents];
      BASE::SveTrajectoryStock_[partitionIndex][k] = BASE::SveTrajectoryStock_[partitionIndex][k + 1];
      BASE::sTrajectoryStock_[partitionIndex][k] =
          BASE::sTrajectoryStock_[partitionIndex][k + 1] + BASE::qFinalStock_[partitionIndex][remainingEvents];

    } else {
      _Gm = (AmConstrainedTrajectoryStock_[partitionIndex][k] + BmConstrainedTrajectoryStock_[partitionIndex][k] * _LmConstrained)
                .transpose();
      _Gv = (QvConstrainedTrajectoryStock_[partitionIndex][k] +
             _LmConstrained.transpose() * RvConstrainedTrajectoryStock_[partitionIndex][k]);
      _Gve = (CmProjectedTrajectoryStock_[partitionIndex][k] + _Lm).transpose() * BASE::RmTrajectoryStock_[partitionIndex][k] *
             EvProjectedTrajectoryStock_[partitionIndex][k];

      BASE::SvTrajectoryStock_[partitionIndex][k] =
          integrateIncrement(workerIndex, _Gm, _Gv, BASE::SvTrajectoryStock_[partitionIndex][k + 1], -deltaT);
      BASE::SveTrajectoryStock_[partitionIndex][k] =
          integrateIncrement(workerIndex, _Gm, _Gve, BASE::SveTrajectoryStock_[partitionIndex][k + 1], -deltaT);
      BASE::sTrajectoryStock_[partitionIndex][k] =
          BASE::sTrajectoryStock_[partitionIndex][k + 1] - deltaT * BASE::qTrajectoryStock_[partitionIndex][k];
    }

    LvConstrainedFunc_(partitionIndex, k, _LvConstrained);
    LveConstrainedFunc_(partitionIndex, k, _LveConstrained);

    // controller
    ControllerFunc_(partitionIndex, k, constraintStepSize, _LmConstrained, _LvConstrained, _LveConstrained);
  }

  // testing the numerical stability
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      // checking the numerical stability of the Riccati equations
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sv is unstable.");
        }
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sve is unstable.");
        }
        if (!BASE::sTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("s is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "Sve[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"
                    << BASE::sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
        }
        throw;
      }

      // checking the numerical stability of the controller parameters
      try {
        if (BASE::nominalControllersStock_[partitionIndex].gainArray_[k].hasNaN()) {
          throw std::runtime_error("Feedback gains are unstable.");
        }
        if (BASE::nominalControllersStock_[partitionIndex].biasArray_[k].hasNaN()) {
          throw std::runtime_error("uff gains are unstable.");
        }
        if (BASE::nominalControllersStock_[partitionIndex].deltaBiasArray_[k].hasNaN()) {
          throw std::runtime_error("deltaUff is unstable.");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[partitionIndex].timeStamp_[k] << " [sec]."
                  << std::endl;
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_Settings& SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_Hamiltonian<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer(const size_t& numPartitions) {
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
  BmConstrainedTrajectoryStock_.resize(numPartitions);
  PmConstrainedTrajectoryStock_.resize(numPartitions);
  RvConstrainedTrajectoryStock_.resize(numPartitions);
  RmInverseTrajectoryStock_.resize(numPartitions);
}

}  // namespace ocs2
