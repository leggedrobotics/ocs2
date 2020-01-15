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

#include <ocs2_ddp/SLQ.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ<STATE_DIM, INPUT_DIM>::SLQ(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                               const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                               const operating_trajectories_base_t* operatingTrajectoriesPtr,
                               const SLQ_Settings& settings /*= SLQ_Settings()*/,
                               std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                               const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           heuristicsFunctionPtr, "SLQ", std::move(logicRulesPtr)),
      settings_(settings) {
  // Riccati Solver
  errorEquationPtrStock_.clear();
  errorEquationPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  errorIntegratorPtrStock_.clear();
  errorIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  IntegratorType integratorType = settings_.RiccatiIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER) {
    throw(
        std::runtime_error("Unsupported Riccati equation integrator type: " + integrator_type::toString(settings_.RiccatiIntegratorType_)));
  }

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    errorEquationPtrStock_.emplace_back(new error_equation_t);
    ;
    bool preComputeRiccatiTerms_ = settings_.preComputeRiccatiTerms_ && (BASE::ddpSettings_.strategy_ == DDP_Strategy::LINE_SEARCH);
    riccatiEquationsPtrStock_.emplace_back(new riccati_equations_t(preComputeRiccatiTerms_));

    errorIntegratorPtrStock_.emplace_back(newIntegrator<STATE_DIM>(integratorType));
    riccatiIntegratorPtrStock_.emplace_back(newIntegrator<riccati_equations_t::S_DIM_>(integratorType));
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
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
    RmInvConstrainedCholTrajectoryStock_[i].resize(N);
  }  // end of i loop

  // base method should be called after the resizes
  BASE::approximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  // unconstrained LQ problem
  BASE::approximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

  // augment the cost function
  BASE::augmentCostWorker(workerIndex, BASE::modelDataTrajectoriesStock_[partitionIndex][timeIndex]);

  // Compute R inverse after inequalities are added to the cost
  // Compute it through the cholesky decomposition as we can reuse the factorization later on
  dynamic_matrix_t RinvChol, DdaggerT_R_Ddagger_Chol;
  LinearAlgebra::computeLinvTLinv(BASE::modelDataTrajectoriesStock_[partitionIndex][timeIndex].costInputSecondDerivative_,
                                  BASE::RmCholeskyUpperTrajectoryStock_[partitionIndex][timeIndex], RinvChol);
  BASE::RmInverseTrajectoryStock_[partitionIndex][timeIndex].noalias() = RinvChol * RinvChol.transpose();

  const auto nc1 = BASE::modelDataTrajectoriesStock_[partitionIndex][timeIndex].numStateInputEqConstr_;
  dynamic_matrix_t DmDager;
  if (nc1 == 0) {
    DmDagerTrajectoryStock_[partitionIndex][timeIndex].setZero();
    RmInvConstrainedCholTrajectoryStock_[partitionIndex][timeIndex] = RinvChol;
  } else {
    const auto& Dm = BASE::modelDataTrajectoriesStock_[partitionIndex][timeIndex].stateInputEqConstrInputDerivative_;
    // check numerics
    if (BASE::ddpSettings_.checkNumericalStability_) {
      if (LinearAlgebra::rank(Dm) != nc1) {
        BASE::printString(">>> WARNING: The state-input constraints are rank deficient (at time " +
                          std::to_string(BASE::nominalTimeTrajectoriesStock_[partitionIndex][timeIndex]) + ")!");
      }
    }
    // Constraint projectors are obtained at once
    ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol,
                                                     RmInvConstrainedCholTrajectoryStock_[partitionIndex][timeIndex]);
    // projection
    DmDagerTrajectoryStock_[partitionIndex][timeIndex].leftCols(nc1) = DmDager;
  }

  // project unconstrained LQ coefficients to constrained ones
  projectLQWorker(workerIndex, partitionIndex, timeIndex, DmDager, DdaggerT_R_Ddagger_Chol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::projectLQWorker(size_t workerIndex, size_t i, size_t k, const dynamic_matrix_t& DmDager,
                                                const dynamic_matrix_t& DdaggerT_R_Ddagger_Chol) {
  // constraint type 1 coefficients
  const auto nc1 = BASE::modelDataTrajectoriesStock_[i][k].numStateInputEqConstr_;
  if (nc1 == 0) {
    EvProjectedTrajectoryStock_[i][k].setZero();
    CmProjectedTrajectoryStock_[i][k].setZero();
    DmProjectedTrajectoryStock_[i][k].setZero();
    AmConstrainedTrajectoryStock_[i][k] = BASE::modelDataTrajectoriesStock_[i][k].dynamicsStateDerivative_;
    QmConstrainedTrajectoryStock_[i][k] = BASE::modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_;
    QvConstrainedTrajectoryStock_[i][k] = BASE::modelDataTrajectoriesStock_[i][k].costStateDerivative_;
  } else {
    const auto& Cm = BASE::modelDataTrajectoriesStock_[i][k].stateInputEqConstrStateDerivative_;
    const auto& Dm = BASE::modelDataTrajectoriesStock_[i][k].stateInputEqConstrInputDerivative_;

    // Projected Constraints
    EvProjectedTrajectoryStock_[i][k].noalias() = DmDager * BASE::modelDataTrajectoriesStock_[i][k].stateInputEqConstr_;
    CmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Cm;
    DmProjectedTrajectoryStock_[i][k].noalias() = DmDager * Dm;

    // Am constrained
    AmConstrainedTrajectoryStock_[i][k] = BASE::modelDataTrajectoriesStock_[i][k].dynamicsStateDerivative_;
    AmConstrainedTrajectoryStock_[i][k].noalias() -=
        BASE::modelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_ * CmProjectedTrajectoryStock_[i][k];

    // Qm constrained
    state_matrix_t PmTransDmDagerCm =
        BASE::modelDataTrajectoriesStock_[i][k].costInputStateDerivative_.transpose() * CmProjectedTrajectoryStock_[i][k];
    QmConstrainedTrajectoryStock_[i][k] =
        BASE::modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_ - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
    dynamic_matrix_t Cm_RProjected_Cm_Chol = DdaggerT_R_Ddagger_Chol.transpose() * Cm;
    QmConstrainedTrajectoryStock_[i][k].noalias() += Cm_RProjected_Cm_Chol.transpose() * Cm_RProjected_Cm_Chol;

    // Qv constrained
    QvConstrainedTrajectoryStock_[i][k] = BASE::modelDataTrajectoriesStock_[i][k].costStateDerivative_;
    QvConstrainedTrajectoryStock_[i][k].noalias() -=
        CmProjectedTrajectoryStock_[i][k].transpose() * BASE::modelDataTrajectoriesStock_[i][k].costInputDerivative_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const {
  const auto activeSubsystem = lookup::findBoundedActiveIntervalInTimeArray(BASE::partitioningTimes_, time);

  const auto indexAlpha =
      EigenLinearInterpolation<state_vector_t>::timeSegment(time, &BASE::nominalTimeTrajectoriesStock_[activeSubsystem]);

  state_vector_t xNominal;
  EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, xNominal, &BASE::nominalStateTrajectoriesStock_[activeSubsystem]);

  dynamic_matrix_t Bm;
  ModelData::LinearInterpolation::interpolate(indexAlpha, Bm, &BASE::modelDataTrajectoriesStock_[activeSubsystem],
                                              ModelData::dynamicsInputDerivative);

  dynamic_matrix_t Pm;
  ModelData::LinearInterpolation::interpolate(indexAlpha, Pm, &BASE::modelDataTrajectoriesStock_[activeSubsystem],
                                              ModelData::costInputStateDerivative);

  dynamic_vector_t Rv;
  ModelData::LinearInterpolation::interpolate(indexAlpha, Rv, &BASE::modelDataTrajectoriesStock_[activeSubsystem],
                                              ModelData::costInputDerivative);

  dynamic_matrix_t Rm;
  ModelData::LinearInterpolation::interpolate(indexAlpha, Rm, &BASE::modelDataTrajectoriesStock_[activeSubsystem],
                                              ModelData::costInputSecondDerivative);

  input_vector_t EvProjected;
  EigenLinearInterpolation<input_vector_t>::interpolate(indexAlpha, EvProjected, &EvProjectedTrajectoryStock_[activeSubsystem]);

  input_state_matrix_t CmProjected;
  EigenLinearInterpolation<input_state_matrix_t>::interpolate(indexAlpha, CmProjected, &CmProjectedTrajectoryStock_[activeSubsystem]);

  input_constraint1_matrix_t DmDager;
  EigenLinearInterpolation<input_constraint1_matrix_t>::interpolate(indexAlpha, DmDager, &DmDagerTrajectoryStock_[activeSubsystem]);

  state_vector_t costate;
  BASE::getValueFunctionStateDerivative(time, state, costate);

  const auto nc1 = BASE::modelDataTrajectoriesStock_[activeSubsystem][indexAlpha.first].numStateInputEqConstr_;
  state_vector_t deltaX = state - xNominal;
  dynamic_input_matrix_t DmDagerTransRm = DmDager.leftCols(nc1).transpose() * Rm;

  nu = DmDagerTransRm * (CmProjected * deltaX + EvProjected) -
       DmDager.leftCols(nc1).transpose() * (Pm * deltaX + Bm.transpose() * costate + Rv);

  //  alternative computation
  //  nu = DmDager.leftCols(nc1).transpose() * (Rm * DmDager.leftCols(nc1).transpose() * CmProjected * deltaX - Rv - Bm.transpose() *
  //  costate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;
  const auto time = BASE::SsTimeTrajectoryStock_[i][k];

  // local variables
  state_vector_t nominalState;
  input_vector_t nominalInput;
  dynamic_matrix_t Bm;
  dynamic_matrix_t Pm;
  dynamic_vector_t Rv;
  dynamic_matrix_t RmInverse;
  input_vector_t EvProjected;
  input_state_matrix_t CmProjected;
  input_matrix_t DmProjected;

  // interpolate
  const auto indexAlpha = EigenLinearInterpolation<state_vector_t>::timeSegment(time, &(BASE::nominalTimeTrajectoriesStock_[i]));
  EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, nominalState, &(BASE::nominalStateTrajectoriesStock_[i]));
  EigenLinearInterpolation<input_vector_t>::interpolate(indexAlpha, nominalInput, &(BASE::nominalInputTrajectoriesStock_[i]));

  ModelData::LinearInterpolation::interpolate(indexAlpha, Bm, &BASE::modelDataTrajectoriesStock_[i], ModelData::dynamicsInputDerivative);
  ModelData::LinearInterpolation::interpolate(indexAlpha, Pm, &BASE::modelDataTrajectoriesStock_[i], ModelData::costInputStateDerivative);
  ModelData::LinearInterpolation::interpolate(indexAlpha, Rv, &BASE::modelDataTrajectoriesStock_[i], ModelData::costInputDerivative);
  EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, RmInverse, &(BASE::RmInverseTrajectoryStock_[i]));
  EigenLinearInterpolation<input_vector_t>::interpolate(indexAlpha, EvProjected, &(EvProjectedTrajectoryStock_[i]));
  EigenLinearInterpolation<input_state_matrix_t>::interpolate(indexAlpha, CmProjected, &(CmProjectedTrajectoryStock_[i]));
  EigenLinearInterpolation<input_matrix_t>::interpolate(indexAlpha, DmProjected, &(DmProjectedTrajectoryStock_[i]));

  // Lm
  Pm.noalias() += Bm.transpose() * BASE::SmTrajectoryStock_[i][k];  // avoid temporary in the product
  input_state_matrix_t Lm = RmInverse * Pm;

  // Lv, Lve
  Rv.noalias() += Bm.transpose() * BASE::SvTrajectoryStock_[i][k];
  input_vector_t Lv = RmInverse * Rv;
  input_vector_t Lve = RmInverse * (Bm.transpose() * BASE::SveTrajectoryStock_[i][k]);

  input_matrix_t DmNullProjection = input_matrix_t::Identity() - DmProjected;

  // Feedback gains
  BASE::nominalControllersStock_[i].gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k].noalias() -= DmNullProjection * Lm;

  // Bias input
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k] * nominalState -
                                                    BASE::ddpSettings_.constraintStepSize_ * (DmNullProjection * Lve + EvProjected);
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
void SLQ<STATE_DIM, INPUT_DIM>::computeRiccatiModificationTermsWorker(size_t workerIndex, size_t i, size_t k) {
  switch (BASE::ddpSettings_.strategy_) {
    case DDP_Strategy::LINE_SEARCH: {
      auto& Qm = BASE::modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_;
      auto& Pm = BASE::modelDataTrajectoriesStock_[i][k].costInputStateDerivative_;
      auto& Rinv = BASE::RmInverseTrajectoryStock_[i][k];
      state_matrix_t Q_minus_PTRinvP = Qm - (Pm.transpose() * Rinv) * Pm;
      BASE::riccatiModificationStock_[i].deltaQmTrajectory_[k] = Q_minus_PTRinvP;
      BASE::shiftHessian(BASE::riccatiModificationStock_[i].deltaQmTrajectory_[k]);
      BASE::riccatiModificationStock_[i].deltaQmTrajectory_[k] -= Q_minus_PTRinvP;
      BASE::riccatiModificationStock_[i].deltaRmTrajectory_[k].setZero(INPUT_DIM, INPUT_DIM);
      BASE::riccatiModificationStock_[i].deltaPmTrajectory_[k].setZero(INPUT_DIM, STATE_DIM);
      break;
    }
    case DDP_Strategy::LEVENBERG_MARQUARDT: {
      const auto& Am = BASE::modelDataTrajectoriesStock_[i][k].dynamicsStateDerivative_;
      const auto& Bm = BASE::modelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_;
      BASE::riccatiModificationStock_[i].deltaQmTrajectory_[k].setZero(STATE_DIM, STATE_DIM);
      BASE::riccatiModificationStock_[i].deltaRmTrajectory_[k] = BASE::levenbergMarquardtImpl_.riccatiMultiple * Bm.transpose() * Bm;
      BASE::riccatiModificationStock_[i].deltaPmTrajectory_[k] = BASE::levenbergMarquardtImpl_.riccatiMultiple * Bm.transpose() * Am;
      break;
    }
  }  // end of switch-case
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::riccatiSolverTask() {
  size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

  for (int i = BASE::endingIndicesRiccatiWorker_[taskId]; i >= BASE::startingIndicesRiccatiWorker_[taskId]; i--) {
    // for inactive subsystems
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      BASE::SsTimeTrajectoryStock_[i].clear();
      BASE::SsNormalizedTimeTrajectoryStock_[i].clear();
      BASE::SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
      BASE::SmTrajectoryStock_[i].clear();
      BASE::SvTrajectoryStock_[i].clear();
      BASE::SveTrajectoryStock_[i].clear();
      BASE::sTrajectoryStock_[i].clear();

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i].setZero();
        BASE::SvFinalStock_[i].setZero();
        BASE::SveFinalStock_[i].setZero();
        BASE::sFinalStock_[i] = 0.0;
        BASE::xFinalStock_[i].setZero();
      }

    } else {
      state_matrix_t SmFinal;
      state_vector_t SvFinal;
      state_vector_t SveFinal;
      scalar_t sFinal;
      state_vector_t xFinal;

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        SmFinal = BASE::SmFinalStock_[i];
        SvFinal = BASE::SvFinalStock_[i];
        SveFinal = BASE::SveFinalStock_[i];
        sFinal = BASE::sFinalStock_[i];
        xFinal = BASE::xFinalStock_[i];
      }

      // modify the end subsystem final values based on the cached values for asynchronous run
      if (i == BASE::endingIndicesRiccatiWorker_[taskId] && i < BASE::finalActivePartition_) {
        const state_vector_t deltaState = BASE::nominalStateTrajectoriesStock_[i + 1].front() - xFinal;
        sFinal += deltaState.dot(0.5 * SmFinal * deltaState + SvFinal);
        SvFinal += SmFinal * deltaState;
      }

      // solve the backward pass
      constrainedRiccatiEquationsWorker(taskId, i, SmFinal, SvFinal, sFinal, SveFinal);

      // set the final value for next Riccati equation
      if (i > BASE::initActivePartition_) {
        // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i - 1] = BASE::SmTrajectoryStock_[i].front();
        BASE::SvFinalStock_[i - 1] = BASE::SvTrajectoryStock_[i].front();
        BASE::SveFinalStock_[i - 1] = BASE::SveTrajectoryStock_[i].front();
        BASE::sFinalStock_[i - 1] = BASE::sTrajectoryStock_[i].front();
        BASE::xFinalStock_[i - 1] = BASE::nominalStateTrajectoriesStock_[i].front();
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::constrainedRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal,
                                                                  const state_vector_t& SvFinal, const scalar_t& sFinal,
                                                                  const state_vector_t& SveFinal) {
  // solve Sm, Sv, s
  riccatiEquationsWorker(workerIndex, partitionIndex, SmFinal, SvFinal, sFinal);

  // Solve Sve
  errorRiccatiEquationWorker(workerIndex, partitionIndex, SveFinal);

  // testing the numerical stability of the Riccati equations
  int N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();
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
        if (!BASE::SveTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sve is unstable");
        }
        if (BASE::sTrajectoryStock_[partitionIndex][k] != BASE::sTrajectoryStock_[partitionIndex][k]) {
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
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:  \t" << BASE::sTrajectoryStock_[partitionIndex][kp]
                    << std::endl;
        }
        throw;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& SLQ<STATE_DIM, INPUT_DIM>::settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::setupOptimizer(size_t numPartitions) {
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
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal,
                                                       const state_vector_t& SvFinal, const scalar_t& sFinal) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &BASE::modelDataTrajectoriesStock_[partitionIndex],
      &AmConstrainedTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex],
      &QmConstrainedTrajectoryStock_[partitionIndex], &RmInvConstrainedCholTrajectoryStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::modelDataEventTimesStock_[partitionIndex],
      &BASE::riccatiModificationStock_[partitionIndex]);

  // Const partition containers
  const auto& nominalTimeTrajectory = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  const auto& nominalEventsPastTheEndIndices = BASE::nominalPostEventIndicesStock_[partitionIndex];

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

  if (BASE::ddpSettings_.debugPrintRollout_) {
    std::cerr << std::endl << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Partition: " << partitionIndex << ", backward pass time trajectory";
    std::cerr << std::endl << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    for (size_t k = 0; k < outputN; k++) {
      std::cerr << "k: " << k << ", t = " << std::setprecision(12) << SsTimeTrajectory[k] << "\n";
    }
    std::cerr << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationNominalTime(
    IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator, riccati_equations_t& riccatiEquation,
    const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
    scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices, s_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Normalize time
  SsNormalizedTime.resize(nominalTimeSize);
  for (int k = 0; k < nominalTimeSize; k++) {
    SsNormalizedTime[nominalTimeSize - 1 - k] = -nominalTimeTrajectory[k];
  }

  // Normalized switching time indices, start and end of the partition are added at the beginning and end
  std::vector<int> SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (int k = numEvents - 1; k >= 0; k--) {
    int index = static_cast<int>(nominalEventsPastTheEndIndices[k]);
    SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize - index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize);

  // integrating the Riccati equations
  SsNormalizedEventsPastTheEndIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    typename scalar_array_t::const_iterator beginTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i];
    typename scalar_array_t::const_iterator endTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    Observer<riccati_equations_t::S_DIM_> observer(&allSsTrajectory);
    // solve Riccati equations
    riccatiIntegrator.integrate_times(riccatiEquation, observer, allSsFinal, beginTimeItr, endTimeItr, BASE::ddpSettings_.minTimeStep_,
                                      BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps);

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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationAdaptiveTime(
    IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator, riccati_equations_t& riccatiEquation,
    const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
    scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices, s_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Extract switching times from eventIndices and normalize them
  scalar_array_t SsNormalizedSwitchingTimes;
  SsNormalizedSwitchingTimes.reserve(numEvents + 2);
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.back());
  for (int k = numEvents - 1; k >= 0; k--) {
    size_t index = nominalEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory[index]);
  }
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.front());

  // integrating the Riccati equations
  SsNormalizedTime.reserve(maxNumSteps);
  SsNormalizedEventsPastTheEndIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    scalar_t beginTime = SsNormalizedSwitchingTimes[i];
    scalar_t endTime = SsNormalizedSwitchingTimes[i + 1];

    Observer<riccati_equations_t::S_DIM_> observer(&allSsTrajectory, &SsNormalizedTime);
    // solve Riccati equations
    riccatiIntegrator.integrate_adaptive(riccatiEquation, observer, allSsFinal, beginTime, endTime, BASE::ddpSettings_.minTimeStep_,
                                         BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps);

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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::errorRiccatiEquationWorker(size_t workerIndex, size_t partitionIndex, const state_vector_t& SveFinal) {
  // Const partition containers
  const auto& nominalTimeTrajectory = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  const auto& SsNormalizedTime = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex];
  const auto& SsNormalizedEventsPastTheEndIndices = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex];
  const auto& SsTimeTrajectory = BASE::SsTimeTrajectoryStock_[partitionIndex];
  const auto& SmTrajectory = BASE::SmTrajectoryStock_[partitionIndex];

  // modified partition containers
  auto& SveTrajectory = BASE::SveTrajectoryStock_[partitionIndex];

  /*
   * Type_1 constraints error correction compensation
   */
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int ssTimeSize = SsNormalizedTime.size();
  const int numEvents = SsNormalizedEventsPastTheEndIndices.size();

  // Skip calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation
  if (BASE::ddpSettings_.simulationIsConstrained_) {
    SveTrajectory.resize(ssTimeSize, state_vector_t::Zero());
    return;
  }

  /*
   * Calculating the coefficients of the error equation
   */
  state_vector_array_t GvTrajectory(nominalTimeSize);
  state_matrix_array_t GmTrajectory(nominalTimeSize);
  state_matrix_t Sm;
  input_state_matrix_t Lm;
  input_vector_t RmEv;
  for (int k = nominalTimeSize - 1; k >= 0; k--) {
    const auto& Pm = BASE::modelDataTrajectoriesStock_[partitionIndex][k].costInputStateDerivative_;
    const auto& Bm = BASE::modelDataTrajectoriesStock_[partitionIndex][k].dynamicsInputDerivative_;
    const auto& Am = AmConstrainedTrajectoryStock_[partitionIndex][k];
    const auto& RmInvChol = RmInvConstrainedCholTrajectoryStock_[partitionIndex][k];
    const auto& RmInv = BASE::RmInverseTrajectoryStock_[partitionIndex][k];
    const auto& Rm = BASE::modelDataTrajectoriesStock_[partitionIndex][k].costInputSecondDerivative_;
    const auto& Ev = EvProjectedTrajectoryStock_[partitionIndex][k];
    const auto& Cm = CmProjectedTrajectoryStock_[partitionIndex][k];

    // Sm
    EigenLinearInterpolation<state_matrix_t>::interpolate(nominalTimeTrajectory[k], Sm, &SsTimeTrajectory, &SmTrajectory);
    // Lm
    Lm = Pm + Bm.transpose() * Sm;

    GmTrajectory[k] = Am;
    GmTrajectory[k].noalias() -= (Bm * RmInvChol) * (RmInvChol.transpose() * Lm);

    RmEv.noalias() = Rm * Ev;
    GvTrajectory[k].noalias() = Cm.transpose() * RmEv;
    GvTrajectory[k].noalias() -= Lm.transpose() * (RmInv.transpose() * RmEv);
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
  for (int k = 0; k < numEvents; k++) {
    const auto index = SsNormalizedEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimesIndices.push_back(index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(ssTimeSize);

  // integrating the Error Riccati equation
  typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
  for (size_t i = 0; i <= numEvents; i++) {
    beginTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i];
    endTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    Observer<STATE_DIM> observer(&SveTrajectory);
    // solve error Riccati equations
    errorIntegratorPtrStock_[workerIndex]->integrate_times(*errorEquationPtrStock_[workerIndex], observer, SveFinalInternal, beginTimeItr,
                                                           endTimeItr, BASE::ddpSettings_.minTimeStep_, BASE::ddpSettings_.absTolODE_,
                                                           BASE::ddpSettings_.relTolODE_, maxNumSteps);

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
