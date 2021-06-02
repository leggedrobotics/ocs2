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

#include <ocs2_core/control/StateBasedLinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::setController(ControllerBase* ctrlPtr) {
  if (ctrlPtr == nullptr) {
    throw std::runtime_error("The controller pointer is null!");
  }
  ctrlPtr_ = ctrlPtr;
  ctrlEventTimes_ = ctrlPtr->controllerEventTimes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateBasedLinearController::computeTrajectorySpreadingInput(scalar_t t, const vector_t& x, const scalar_array_t& ctrlEventTimes,
                                                                     ControllerBase* ctrlPtr) {
  size_t currentMode = static_cast<size_t>(x.tail(1).value());
  size_t numEvents = ctrlEventTimes.size();

  if (numEvents == 0)  // Simple case in which the controller does not contain any events
  {
    return ctrlPtr->computeInput(t, x);
  }

  scalar_t tauMinus = (numEvents > currentMode) ? ctrlEventTimes[currentMode] : ctrlEventTimes.back();
  scalar_t tau = (numEvents > currentMode + 1) ? ctrlEventTimes[currentMode + 1] : ctrlEventTimes.back();
  scalar_t tauPlus = (numEvents > currentMode + 2) ? ctrlEventTimes[currentMode + 2] : ctrlEventTimes.back();

  bool pastAllEvents = (currentMode >= numEvents - 1) && (t > tauMinus);
  const scalar_t eps = numeric_traits::weakEpsilon<scalar_t>();

  if (pastAllEvents) {
    return ctrlPtr->computeInput(t, x);
    // return normal input signal
  } else if (t < tauMinus) {
    // if event happened before the event time for which the controller was designed
    return ctrlPtr->computeInput(tauMinus + 2.0 * eps, x);
    // request input 1 epsilon after the designed event time
  } else if (t > tau) {
    // if event has not happened yet at the event time for which the controller was designed
    return ctrlPtr->computeInput(tau - eps, x);
    // request input 1 epsilon before the designed event time
  }
  // normal case: t > tauMinus && t < tau
  return ctrlPtr->computeInput(t, x);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateBasedLinearController::computeInput(scalar_t t, const vector_t& x) {
  return computeTrajectorySpreadingInput(t, x, ctrlEventTimes_, ctrlPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::concatenate(const ControllerBase* nextController, int index, int length) {
  ctrlPtr_->concatenate(nextController, index, length);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int StateBasedLinearController::size() const {
  return ctrlPtr_->size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType StateBasedLinearController::getType() const {
  return ctrlPtr_->getType();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::clear() {
  ctrlPtr_->clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateBasedLinearController::empty() const {
  return ctrlPtr_->empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::display() const {
  ctrlPtr_->display();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateBasedLinearController* StateBasedLinearController::clone() const {
  return new StateBasedLinearController(*this);
}

}  // namespace ocs2
