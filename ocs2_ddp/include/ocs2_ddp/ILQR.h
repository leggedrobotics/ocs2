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

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>

#include "GaussNewtonDDP.h"
#include "riccati_equations/DiscreteTimeRiccatiEquations.h"

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread ILQR.
 */
class ILQR : public GaussNewtonDDP {
 public:
  /**
   * Constructor
   *
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  ILQR(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
       const Initializer& initializer);

  /**
   * Default destructor.
   */
  ~ILQR() override = default;

 protected:
  scalar_t solveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) override;

  void riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                              const ScalarFunctionQuadraticApproximation& finalValueFunction) override;

  void calculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                 LinearController& dstController) override;

  matrix_t computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const override;

  void approximateIntermediateLQ(PrimalDataContainer& primalData) override;

  /**
   * Calculates the discrete-time LQ approximation from the continuous-time LQ approximation.
   *
   * @param [in] system: system dynamic.
   * @param [in] time: time t_k.
   * @param [in] state: state x_k.
   * @param [in] input: input u_k.
   * @param [in] timeStep: Time step between the x_{k} and x_{k+1}.
   * @param [in] continuousTimeModelData: continuous time model data.
   * @param [out] modelData: Discretized mode data.
   */
  void discreteLQWorker(SystemDynamicsBase& system, scalar_t time, const vector_t& state, const vector_t& input, scalar_t timeStep,
                        const ModelData& continuousTimeModelData, ModelData& modelData);

  /****************
   *** Variables **
   ****************/
  matrix_array_t projectedKmTrajectoryStock_;  // projected feedback
  vector_array_t projectedLvTrajectoryStock_;  // projected feedforward

  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<std::unique_ptr<DiscreteTimeRiccatiEquations>> riccatiEquationsPtrStock_;
};

}  // namespace ocs2
