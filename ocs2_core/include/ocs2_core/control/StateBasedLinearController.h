#pragma once

#include "ocs2_core/control/ControllerBase.h"
#include "ocs2_core/control/LinearController.h"
#include "ocs2_core/misc/LinearInterpolation.h"

#include <iomanip>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>

class stateBasedLinearController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using size_array_t = typename dimensions_t::size_array_t;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using float_array_t = typename Base::float_array_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;

  /**
   * Default constructor.
   */
  stateBasedLinearController() = default;

  /**
   * Default destructor.
   */
  ~stateBasedLinearController() override = default;

  /**
   * Sets the provided controller pointer which provides the input signal
   * At the same time the eventtimes of the designed controller are computed
   *
   * @param[in] ctrlPtr: pointer to the provided controller
   */
  void setController(controller_t* ctrlPtr) {
    if (!ctrlPtr) {
      throw std::runtime_error("The controller pointer is null!");
    }
    ctrlPtr_ = ctrlPtr;
    ctrlEventTimes_ = ctrlPtr->controllerEventTimes();
  }

  /**
   * Computes the control input based on the trajectory spreading scheme.
   *
   * @param [in] t: current time at which input is requested
   * @param [in] x: current state at which input is requested
   * @param [in] ctrlEventTimes: array containing eventTimes around which the controller was designed
   * @param [in] ctrlPtr: Pointer to the actual controller
   *
   * @retrun
   */
  static input_vector_t computeTrajectorySpreadingInput(const scalar_t& t, const state_vector_t& x, const scalar_array_t& ctrlEventTimes,
                                                        controller_t* ctrlPtr) {
    size_t currentMode = x.tail(1).value();
    size_t numEvents = ctrlEventTimes.size();

    if (numEvents == 0)  // Simple case in which the controller does not contain any events
    {
      return ctrlPtr->computeInput(t, x);
    }

    scalar_t tauMinus = (numEvents > currentMode) ? ctrlEventTimes[currentMode] : ctrlEventTimes.back();
    scalar_t tau = (numEvents > currentMode + 1) ? ctrlEventTimes[currentMode + 1] : ctrlEventTimes.back();
    scalar_t tauPlus = (numEvents > currentMode + 2) ? ctrlEventTimes[currentMode + 2] : ctrlEventTimes.back();

    bool pastAllEvents = (currentMode >= numEvents - 1) && (t > tauMinus);
    const scalar_t eps = OCS2NumericTraits<scalar_t>::weakEpsilon();

    if ((t > tauMinus && t < tau) || pastAllEvents) {  // normal case
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
  }

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    return computeTrajectorySpreadingInput(t, x, ctrlEventTimes_, ctrlPtr_);
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    ctrlPtr_->flatten(timeArray, flatArray2);
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    ctrlPtr_->unFlatten(timeArray, flatArray2);
  }

  void concatenate(const Base* nextController, int index, int length) override { ctrlPtr_->concatenate(nextController, index, length); }

  int size() const override { return ctrlPtr_->size(); }

  ControllerType getType() const override { return ctrlPtr_->getType(); }

  void clear() override { ctrlPtr_->clear(); }

  void setZero() override { ctrlPtr_->setZero(); }

  bool empty() const override { return ctrlPtr_->empty(); }

  void display() const override { ctrlPtr_->display(); }

  stateBasedLinearController* clone() const override { return new stateBasedLinearController(*this); }

 private:
  controller_t* ctrlPtr_ = nullptr;
  scalar_array_t ctrlEventTimes_;
};

}  // namespace ocs2
