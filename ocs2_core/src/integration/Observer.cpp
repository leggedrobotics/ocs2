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

#include <ocs2_core/integration/Observer.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Observer::Observer(vector_array_t* stateTrajectoryPtr /*= nullptr*/, scalar_array_t* timeTrajectoryPtr /*= nullptr*/,
                   std::vector<ModelDataBase>* modelDataTrajectoryPtr /*= nullptr*/)
    : timeTrajectoryPtr_(timeTrajectoryPtr), stateTrajectoryPtr_(stateTrajectoryPtr), modelDataTrajectoryPtr_(modelDataTrajectoryPtr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Observer::observe(OdeBase& system, const vector_t& state, const scalar_t time) {
  // Store data
  if (stateTrajectoryPtr_ != nullptr) {
    stateTrajectoryPtr_->push_back(state);
  }
  if (timeTrajectoryPtr_ != nullptr) {
    timeTrajectoryPtr_->push_back(time);
  }

  // extract cached model data
  if (modelDataTrajectoryPtr_) {
    // check for initial call
    if (system.endModelDataIterator() == system.beginModelDataIterator()) {
      // since the flow map has not been called so far, call it once.
      // Note: This is a workaround for boost::odeint() as it calls observer() before flowMap()
      vector_t dxdt;
      system.computeFlowMap(time, state, dxdt);
    }
    // get model data from current time step, search starting from most recent cache entry
    std::vector<ModelDataBase>::iterator modelData_i = system.endModelDataIterator();
    while (modelData_i != system.beginModelDataIterator()) {
      --modelData_i;
      if (numerics::almost_eq(modelData_i->time_, time)) {
        modelDataTrajectoryPtr_->emplace_back(std::move(*modelData_i));
        break;
      }
    }
    system.clearModelDataArray();
  }
}

}  // namespace ocs2
