/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection) {
  ocp.finalEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.finalInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection) {
  ocp.preJumpEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.preJumpInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time,
                                                MultiplierCollection& multiplierCollection) {
  ocp.stateEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.stateInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
  ocp.equalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateInputEq);
  ocp.inequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateInputIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                     MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.finalEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.finalInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                       MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.preJumpEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.preJumpInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.stateEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.stateInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
  ocp.equalityLagrangianPtr->updateLagrangian(time, state, input, metricsCollection.stateInputEqLagrangian,
                                              multiplierCollection.stateInputEq);
  ocp.inequalityLagrangianPtr->updateLagrangian(time, state, input, metricsCollection.stateInputIneqLagrangian,
                                                multiplierCollection.stateInputIneq);
}

}  // namespace ocs2
