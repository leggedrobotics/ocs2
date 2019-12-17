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

#include "ocs2_core/control/ControllerBase.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * LinearController implements a time and state dependent controller of the
 * form u[x,t] = k[t] * x + uff[t]
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using size_array_t = typename dimensions_t::size_array_t;
  using float_array_t = typename Base::float_array_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

  using self_t = LinearController<STATE_DIM, INPUT_DIM>;
  using array_t = std::vector<self_t, Eigen::aligned_allocator<self_t>>;

  /**
   * @brief Default constructor leaves object uninitialized
   */
  LinearController() = default;

  /**
   * @brief Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  LinearController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerBias,
                   const input_state_matrix_array_t& controllerGain)
      : LinearController() {
    setController(controllerTime, controllerBias, controllerGain);
  }

  /**
   * @brief Copy constructor
   * @param other LinearController object to copy from
   */
  LinearController(const LinearController& other) : LinearController(other.timeStamp_, other.biasArray_, other.gainArray_) {
    deltaBiasArray_ = other.deltaBiasArray_;
  }

  /**
   * @brief LinearController move constructor -- not implemented for now
   * @param other LinearController object to move from
   * @todo Implement
   */
  LinearController(LinearController&& other) = delete;

  /**
   * @brief Copy assignment (copy and swap idiom)
   * @param other LinearController object to assign from
   */
  LinearController& operator=(LinearController other) {
    other.swap(*this);
    return *this;
  }

  /**
   * @brief Move assignment -- not implemented for now
   * @param other LinearController object to assign from
   * @todo Implement
   */
  LinearController& operator=(LinearController&& other) = delete;

  /**
   * @brief Destructor
   */
  ~LinearController() override = default;

  LinearController* clone() const override { return new LinearController(*this); }

  /**
   * @brief setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  void setController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerBias,
                     const input_state_matrix_array_t& controllerGain) {
    timeStamp_ = controllerTime;
    biasArray_ = controllerBias;
    gainArray_ = controllerGain;
  }

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    input_vector_t uff;
    const auto indexAlpha = EigenLinearInterpolation<input_vector_t>::interpolate(t, uff, &timeStamp_, &biasArray_);

    input_state_matrix_t k;
    EigenLinearInterpolation<input_state_matrix_t>::interpolate(indexAlpha, k, &gainArray_);

    uff.noalias() += k * x;
    return uff;
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    const auto timeSize = timeArray.size();
    const auto dataSize = flatArray2.size();

    if (timeSize != dataSize) {
      throw std::runtime_error("timeSize and dataSize must be equal in flatten method.");
    }

    for (size_t i = 0; i < timeSize; i++) {
      flattenSingle(timeArray[i], *(flatArray2[i]));
    }
  }

  void flattenSingle(scalar_t time, float_array_t& flatArray) const {
    flatArray.clear();
    flatArray.resize(INPUT_DIM + INPUT_DIM * STATE_DIM);

    input_vector_t uff;
    const auto indexAlpha = EigenLinearInterpolation<input_vector_t>::interpolate(time, uff, &timeStamp_, &biasArray_);

    input_state_matrix_t k;
    EigenLinearInterpolation<input_state_matrix_t>::interpolate(indexAlpha, k, &gainArray_);

    for (int i = 0; i < INPUT_DIM; i++) {  // i loops through input dim
      flatArray[i * (STATE_DIM + 1) + 0] = static_cast<float>(uff(i));
      for (int j = 0; j < STATE_DIM; j++) {  // j loops through state dim
        flatArray[i * (STATE_DIM + 1) + j + 1] = static_cast<float>(k(i, j));
      }
    }
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    if (flatArray2[0]->size() != INPUT_DIM + INPUT_DIM * STATE_DIM) {
      throw std::runtime_error("LinearController::unFlatten received array of wrong length.");
    }

    timeStamp_ = timeArray;

    biasArray_.clear();
    biasArray_.reserve(flatArray2.size());
    gainArray_.clear();
    gainArray_.reserve(flatArray2.size());

    for (const auto& arr : flatArray2) {  // loop through time
      biasArray_.emplace_back(input_vector_t::Zero());
      gainArray_.emplace_back(input_state_matrix_t::Zero());

      for (int i = 0; i < INPUT_DIM; i++) {  // loop through input dim
        biasArray_.back()(i) = static_cast<scalar_t>((*arr)[i * (STATE_DIM + 1) + 0]);
        gainArray_.back().row(i) =
            Eigen::Map<const Eigen::Matrix<float, 1, STATE_DIM>>(&((*arr)[i * (STATE_DIM + 1) + 1]), STATE_DIM).template cast<scalar_t>();
      }
    }
  }

  void concatenate(const Base* nextController, int index, int length) override {
    if (auto nextLinCtrl = dynamic_cast<const LinearController*>(nextController)) {
      if (!timeStamp_.empty() && timeStamp_.back() > nextLinCtrl->timeStamp_.front()) {
        throw std::runtime_error("Concatenate requires that the nextController comes later in time.");
      }
      int last = index + length;
      timeStamp_.insert(timeStamp_.end(), nextLinCtrl->timeStamp_.begin() + index, nextLinCtrl->timeStamp_.begin() + last);
      biasArray_.insert(biasArray_.end(), nextLinCtrl->biasArray_.begin() + index, nextLinCtrl->biasArray_.begin() + last);
      deltaBiasArray_.insert(deltaBiasArray_.end(), nextLinCtrl->deltaBiasArray_.begin() + index,
                             nextLinCtrl->deltaBiasArray_.begin() + last);
      gainArray_.insert(gainArray_.end(), nextLinCtrl->gainArray_.begin() + index, nextLinCtrl->gainArray_.begin() + last);
    } else {
      throw std::runtime_error("Concatenate only works with controllers of the same type.");
    }
  }

  int size() const override { return timeStamp_.size(); }

  ControllerType getType() const override { return ControllerType::LINEAR; }

  void clear() override {
    timeStamp_.clear();
    biasArray_.clear();
    gainArray_.clear();
  }

  void setZero() override {
    std::fill(biasArray_.begin(), biasArray_.end(), input_vector_t::Zero());
    std::fill(deltaBiasArray_.begin(), deltaBiasArray_.end(), input_vector_t::Zero());
    std::fill(gainArray_.begin(), gainArray_.end(), input_state_matrix_t::Zero());
  }

  bool empty() const override { return timeStamp_.empty(); }

  void display() const override {
    for (size_t k = 0; k < timeStamp_.size(); k++) {
      std::cerr << "k: " << k << std::endl;
      std::cerr << "time: " << timeStamp_[k] << std::endl;
      std::cerr << "bias: " << biasArray_[k].transpose() << std::endl;
      std::cerr << "gain: " << gainArray_[k] << std::endl;
    }
  }

  /**
   * @brief Swap data with other object
   * @param other the object to be swapped with
   */
  virtual void swap(LinearController<STATE_DIM, INPUT_DIM>& other) {
    using std::swap;  // enable ADL

    swap(timeStamp_, other.timeStamp_);
    swap(biasArray_, other.biasArray_);
    swap(deltaBiasArray_, other.deltaBiasArray_);
    swap(gainArray_, other.gainArray_);
  }

  /**
   * @brief getFeedbackGain: Extracts the feedback matrix at the requested time
   * @param[in] time
   * @param[out] gain: linear feedback gain
   */
  void getFeedbackGain(scalar_t time, input_state_matrix_t& gain) const {
    EigenLinearInterpolation<input_state_matrix_t>::interpolate(time, gain, &timeStamp_, &gainArray_);
  }

  /**
   * @brief getFeedbackGain: Extracts the bias term at the requested time
   * @param[in] time
   * @param[out] bias: the controller bias term
   */
  void getBias(scalar_t time, input_vector_t& bias) const {
    EigenLinearInterpolation<input_vector_t>::interpolate(time, bias, &timeStamp_, &biasArray_);
  }

  scalar_array_t controllerEventTimes() const override {
    // simple controller case
    if (timeStamp_.size() < 2) {
      return scalar_array_t(0);
    }

    scalar_array_t eventTimes{0.0};
    scalar_t lastevent = timeStamp_.front();
    for (int i = 0; i < timeStamp_.size() - 1; i++) {
      bool eventDetected = timeStamp_[i + 1] - timeStamp_[i] < 2.0 * OCS2NumericTraits<scalar_t>::weakEpsilon();
      const bool sufficientTimeSinceEvent = timeStamp_[i] - lastevent > 2.0 * OCS2NumericTraits<scalar_t>::weakEpsilon();

      if (eventDetected && sufficientTimeSinceEvent) {  // push back event when event is detected
        eventTimes.push_back(timeStamp_[i]);
        lastevent = eventTimes.back();
      } else if (eventDetected) {
        // if event is detected to close to the last event, it is assumed that the earlier event was not an event
        // but was due to the refining steps taken in event detection
        // The last "detected event" is the time the event took place
        eventTimes.back() = timeStamp_[i];
        lastevent = eventTimes.back();
      }
    }
    return eventTimes;
  }

 public:
  scalar_array_t timeStamp_;
  input_vector_array_t biasArray_;
  input_vector_array_t deltaBiasArray_;
  input_state_matrix_array_t gainArray_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(LinearController<STATE_DIM, INPUT_DIM>& a, LinearController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
