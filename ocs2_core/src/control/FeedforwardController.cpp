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

#include <iostream>

#include <ocs2_core/control/FeedforwardController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(size_t stateDim, size_t inputDim) : ControllerBase(stateDim, inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(size_t stateDim, size_t inputDim, const scalar_array_t& controllerTime,
                                             const vector_array_t& controllerFeedforward)
    : FeedforwardController(stateDim, inputDim) {
  setController(controllerTime, controllerFeedforward);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(size_t stateDim, size_t inputDim, const scalar_array_t& controllerTime,
                                             const vector_array_t& stateTrajectory, ControllerBase* controller)
    : FeedforwardController(stateDim, inputDim) {
  timeStamp_ = controllerTime;
  uffArray_.clear();
  uffArray_.reserve(controllerTime.size());

  if (controllerTime.size() != stateTrajectory.size()) {
    throw std::runtime_error("FeedforwardController Constructor: controllerTime and stateTrajectory sizes mismatch.");
  }

  auto iTime = controllerTime.cbegin();
  auto iState = stateTrajectory.cbegin();

  for (; iTime != controllerTime.end() and iState != stateTrajectory.end(); ++iTime, ++iState) {
    uffArray_.emplace_back(controller->computeInput(*iTime, *iState));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(const FeedforwardController& other)
    : FeedforwardController(other.stateDim_, other.inputDim_, other.timeStamp_, other.uffArray_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController& FeedforwardController::operator=(FeedforwardController other) {
  other.swap(*this);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::setController(const scalar_array_t& controllerTime, const vector_array_t& controllerFeedforward) {
  timeStamp_ = controllerTime;
  uffArray_ = controllerFeedforward;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FeedforwardController::computeInput(scalar_t t, const vector_t& x) {
  vector_t uff;
  LinearInterpolation::interpolate(t, uff, &timeStamp_, &uffArray_);
  return uff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
  const auto timeSize = timeArray.size();
  const auto dataSize = flatArray2.size();

  if (timeSize != dataSize) {
    throw std::runtime_error("timeSize (" + std::to_string(timeSize) + ") and dataSize (" + std::to_string(dataSize) +
                             ") must be equal in flatten method.");
  }

  for (size_t i = 0; i < timeSize; i++) {
    flattenSingle(timeArray[i], *(flatArray2[i]));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::flattenSingle(scalar_t time, std::vector<float>& flatArray) const {
  vector_t uff;
  LinearInterpolation::interpolate(time, uff, &timeStamp_, &uffArray_);

  flatArray = std::move(std::vector<float>(uff.data(), uff.data() + inputDim_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::unFlatten(const scalar_array_t& timeArray, const std::vector<std::vector<float> const*>& flatArray2) {
  if (flatArray2[0]->size() != inputDim_) {
    throw std::runtime_error("FeedforwardController::unFlatten received array of wrong length.");
  }

  timeStamp_ = timeArray;

  uffArray_.clear();
  uffArray_.reserve(flatArray2.size());

  for (const auto& arr : flatArray2) {  // loop through time
    uffArray_.emplace_back(Eigen::Map<const Eigen::VectorXf>(arr->data(), inputDim_).cast<scalar_t>());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::concatenate(const ControllerBase* nextController, int index, int length) {
  // TODO(mspieler): why not use nextController->getType() and do static_cast?
  if (auto nextFfwdCtrl = dynamic_cast<const FeedforwardController*>(nextController)) {
    if (!timeStamp_.empty() && timeStamp_.back() > nextFfwdCtrl->timeStamp_.front()) {
      throw std::runtime_error("Concatenate requires that the nextController comes later in time.");
    }
    int last = index + length;
    timeStamp_.insert(timeStamp_.end(), nextFfwdCtrl->timeStamp_.begin() + index, nextFfwdCtrl->timeStamp_.begin() + last);
    uffArray_.insert(uffArray_.end(), nextFfwdCtrl->uffArray_.begin() + index, nextFfwdCtrl->uffArray_.begin() + last);
  } else {
    throw std::runtime_error("Concatenate only works with controllers of the same type.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int FeedforwardController::size() const {
  return timeStamp_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType FeedforwardController::getType() const {
  return ControllerType::FEEDFORWARD;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::clear() {
  timeStamp_.clear();
  uffArray_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::setZero() {
  std::fill(uffArray_.begin(), uffArray_.end(), vector_t::Zero(inputDim_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FeedforwardController::empty() const {
  return timeStamp_.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::swap(FeedforwardController& other) {
  using std::swap;  // enable ADL

  swap(timeStamp_, other.timeStamp_);
  swap(uffArray_, other.uffArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController* FeedforwardController::clone() const {
  return new FeedforwardController(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::display() const {
  std::cerr << *this << '\n';
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(FeedforwardController& a, FeedforwardController& b) {
  a.swap(b);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const FeedforwardController& controller) {
  for (int i = 0; i < controller.timeStamp_.size(); i++) {
    out << "t\t" << controller.timeStamp_[i] << "\tu\t" << controller.uffArray_[i].transpose() << '\n';
  }
  return out;
}

}  // namespace ocs2
