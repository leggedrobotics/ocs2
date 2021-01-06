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

#include <ocs2_core/integration/IntegratorBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntegratorBase::IntegratorBase(std::shared_ptr<SystemEventHandler> eventHandlerPtr /*= nullptr*/)
    : eventHandlerPtr_(std::move(eventHandlerPtr)) {
  if (eventHandlerPtr_ == nullptr) {
    eventHandlerPtr_ = std::make_shared<SystemEventHandler>();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntegratorBase::system_func_t IntegratorBase::systemFunction(OdeBase& system, int maxNumSteps) const {
  return [&system, maxNumSteps](const vector_t& x, vector_t& dxdt, scalar_t t) {
    dxdt = system.computeFlowMap(t, x);
    // max number of function calls
    if (system.incrementNumFunctionCalls() > maxNumSteps) {
      std::stringstream msg;
      msg << "Integration terminated since the maximum number of function calls is reached. State at termination time " << t << ":\n["
          << x.transpose() << "]\n";
      throw std::runtime_error(msg.str());
    }
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::integrateConst(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime,
                                    scalar_t finalTime, scalar_t dt, int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  runIntegrateConst(systemFunction(system, maxNumSteps), callback, initialState, startTime, finalTime, dt);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::integrateAdaptive(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime,
                                       scalar_t finalTime, scalar_t dtInitial /*= 0.01*/, scalar_t AbsTol /*= 1e-6*/,
                                       scalar_t RelTol /*= 1e-3*/, int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  runIntegrateAdaptive(systemFunction(system, maxNumSteps), callback, initialState, startTime, finalTime, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::integrateTimes(OdeBase& system, Observer& observer, const vector_t& initialState,
                                    typename scalar_array_t::const_iterator beginTimeItr,
                                    typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial /*= 0.01*/,
                                    scalar_t AbsTol /*= 1e-6*/, scalar_t RelTol /*= 1e-3*/,
                                    int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  runIntegrateTimes(systemFunction(system, maxNumSteps), callback, initialState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
}

}  // namespace ocs2
