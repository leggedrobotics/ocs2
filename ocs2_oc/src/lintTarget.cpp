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

// approximate model
#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

// multiple_shooting
#include <ocs2_oc/multiple_shooting/Helpers.h>
#include <ocs2_oc/multiple_shooting/Initialization.h>
#include <ocs2_oc/multiple_shooting/LagrangianEvaluation.h>
#include <ocs2_oc/multiple_shooting/MetricsComputation.h>
#include <ocs2_oc/multiple_shooting/PerformanceIndexComputation.h>
#include <ocs2_oc/multiple_shooting/ProjectionMultiplierCoefficients.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>

// oc_data
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>

// oc_problem
#include <ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h>
#include <ocs2_oc/oc_problem/OcpSize.h>
#include <ocs2_oc/oc_problem/OcpToKkt.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h>

// oc_solver
#include <ocs2_oc/oc_solver/SolverBase.h>

// precondition
#include <ocs2_oc/precondition/Ruzi.h>

// synchronized_module
#include <ocs2_oc/synchronized_module/LoopshapingReferenceManager.h>
#include <ocs2_oc/synchronized_module/LoopshapingSynchronizedModule.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_oc/synchronized_module/SolverObserver.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

// rollout
#include <ocs2_oc/rollout/InitializerRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/RolloutSettings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// search_strategy
#include <ocs2_oc/search_strategy/FilterLinesearch.h>

// trajectory_adjustment
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreading.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
