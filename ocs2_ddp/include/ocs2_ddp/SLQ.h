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

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SystemEventHandler.h>

#include "GaussNewtonDDP.h"
#include "SLQ_Settings.h"
#include "riccati_equations/ContinuousTimeRiccatiEquations.h"

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 */
class SLQ final : public GaussNewtonDDP {
 public:
  using BASE = GaussNewtonDDP;

  /**
   * Constructor
   *
   * @param [in] stateDim: State vector dimension
   * @param [in] inputDim: Input vector dimension
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  SLQ(size_t stateDim, size_t inputDim, const RolloutBase* rolloutPtr, const DerivativesBase* systemDerivativesPtr,
      const ConstraintBase* systemConstraintsPtr, const CostFunctionBase* costFunctionPtr,
      const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr, const SLQ_Settings& settings = SLQ_Settings(),
      const CostFunctionBase* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  ~SLQ() override = default;

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  SLQ_Settings& settings();

 protected:
  matrix_t computeHamiltonianHessian(ddp_strategy::type strategy, const ModelDataBase& modelData, const matrix_t& Sm) const override;

  void approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                 const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory,
                                 ModelDataBase::array_t& modelDataTrajectory) override;

  void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  scalar_t solveSequentialRiccatiEquations(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) override;

  void riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const matrix_t& SmFinal, const vector_t& SvFinal,
                              const scalar_t& sFinal) override;

  /**
   * Integrates the riccati equation and generates the value function at the times set in nominal Time Trajectory.
   *
   * @param riccatiIntegrator [in] : Riccati integrator object
   * @param riccatiEquation [in] : Riccati equation object
   * @param nominalTimeTrajectory [in] : time trajectory produced in the forward rollout.
   * @param nominalEventsPastTheEndIndices [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param allSsFinal [in] : Final value of the value function.
   * @param SsNormalizedTime [out] : Time trajectory of the value function.
   * @param SsNormalizedPostEventIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationNominalTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                           const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices,
                                           vector_t allSsFinal, scalar_array_t& SsNormalizedTime,
                                           size_array_t& SsNormalizedPostEventIndices, vector_array_t& allSsTrajectory);

  /**
   * Integrates the riccati equation and freely selects the time nodes for the value function.
   *
   * @param riccatiIntegrator [in] : Riccati integrator object
   * @param riccatiEquation [in] : Riccati equation object
   * @param nominalTimeTrajectory [in] : time trajectory produced in the forward rollout.
   * @param nominalEventsPastTheEndIndices [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param allSsFinal [in] : Final value of the value function.
   * @param SsNormalizedTime [out] : Time trajectory of the value function.
   * @param SsNormalizedPostEventIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationAdaptiveTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                            const scalar_array_t& nominalTimeTrajectory, const size_array_t& nominalEventsPastTheEndIndices,
                                            vector_t allSsFinal, scalar_array_t& SsNormalizedTime,
                                            size_array_t& SsNormalizedPostEventIndices, vector_array_t& allSsTrajectory);

  /****************
   *** Variables **
   ****************/
  SLQ_Settings settings_;

  std::vector<std::shared_ptr<ContinuousTimeRiccatiEquations>> riccatiEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase>> riccatiIntegratorPtrStock_;
};

}  // namespace ocs2
