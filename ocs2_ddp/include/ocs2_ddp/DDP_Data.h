/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include "ocs2_ddp/riccati_equations/RiccatiModification.h"

#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/**
 * Primal data container
 *
 * The design philosophy behind is to keep all member variables consistent. All (time, post, .., modelDataEventTime)
 * trajectories should be the rollout result of the controller
 *
 * There is one exception that breaks the consistency. When using an external controller to initialize the controller, it is obvious that
 * the rest of member variables are not the result of the controller. But they will be cleared and populated when runInit is called.
 */
struct PrimalDataContainer {
  PrimalSolution primalSolution;
  // intermediate model data trajectory
  std::vector<ModelData> modelDataTrajectory;
  // event times model data
  std::vector<ModelData> modelDataEventTimes;

  void swap(PrimalDataContainer& other) {
    primalSolution.swap(other.primalSolution);
    modelDataTrajectory.swap(other.modelDataTrajectory);
    modelDataEventTimes.swap(other.modelDataEventTimes);
  }

  void clear() {
    primalSolution.clear();
    modelDataTrajectory.clear();
    modelDataEventTimes.clear();
  }
};

/**
 * Dual data container
 *
 * The design philosophy behind is to keep all member variables consistent. valueFunctionTrajectory is the direct result of
 * (projectedModelData,riccatiModification) trajectories.
 *
 */
struct DualDataContainer {
  // projected model data trajectory
  std::vector<ModelData> projectedModelDataTrajectory;

  // Riccati modification
  std::vector<riccati_modification::Data> riccatiModificationTrajectory;

  // Riccati solution coefficients
  std::vector<ScalarFunctionQuadraticApproximation> valueFunctionTrajectory;

  inline void swap(DualDataContainer& other) {
    projectedModelDataTrajectory.swap(other.projectedModelDataTrajectory);
    riccatiModificationTrajectory.swap(other.riccatiModificationTrajectory);
    valueFunctionTrajectory.swap(other.valueFunctionTrajectory);
  }

  inline void clear() {
    projectedModelDataTrajectory.clear();
    riccatiModificationTrajectory.clear();
    valueFunctionTrajectory.clear();
  }
};

}  // namespace ocs2
