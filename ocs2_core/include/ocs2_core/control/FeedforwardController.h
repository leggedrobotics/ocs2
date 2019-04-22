#pragma once

#include "ocs2_core/control/Controller.h"

#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * FeedforwardController provides a time-dependent control law without state-dependent feedback.
 * Commonly, this is used to wrap around a more general contoller and extract only the feedforward portion.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class FeedforwardController : public Controller<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Controller<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using state_vector_array_t = typename dimensions_t::state_vector_array_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

  using self_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
  using array_t = std::vector<self_t, Eigen::aligned_allocator<self_t>>;

  /**
   * @brief Default constructor leaves object uninitialized
   */
  FeedforwardController() : Base(), linInterpolateUff_(&time_, &uff_) {}

  /**
   * @brief Constructor initializes all required members of the controller
   */
  FeedforwardController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward)
      : FeedforwardController() {
    setController(controllerTime, controllerFeedforward);
  }

  /**
   * @brief Constructor to initialize the feedforward input data with a general controller rolled-out along a nominal stateTrajectory
   * @param controllerTime the times for the rollout
   * @param stateTrajectory the states for the rollout
   * @param controller the controller to extract the feedforward controls from during a rollout
   */
  FeedforwardController(const scalar_array_t& controllerTime, const state_vector_array_t& stateTrajectory, Base* controller):
  FeedforwardController() {
    time_ = controllerTime;
    uff_.clear();
    uff_.reserve(controllerTime.size());

    if(controllerTime.size() != stateTrajectory.size()){
      throw std::runtime_error("FeedforwardController Constructor: controllerTime and stateTrajectory sizes mismatch");
    }

    auto iTime = controllerTime.cbegin();
    auto iState = stateTrajectory.cbegin();

    for(; iTime != controllerTime.end() and iState != stateTrajectory.end(); ++iTime, ++iState){
      uff_.emplace_back(controller->computeInput(*iTime, *iState));
    }
  }

  /**
   * @brief Copy constructor
   * @param other FeedforwardController object to copy from
   */
  FeedforwardController(const FeedforwardController& other) : FeedforwardController(other.time_, other.uff_) {}

  /**
   * @brief Move constructor -- not implemented for now
   * @param other FeedforwardController object to move from
   * @todo Implement
   */
  FeedforwardController(FeedforwardController&& other) = delete;

  /**
   * @brief Copy assignment (copy and swap idiom)
   * @param other FeedforwardController object to assign from
   */
  FeedforwardController& operator=(FeedforwardController other) {
    other.swap(*this);
    return *this;
  }

  /**
   * @brief Move assignment -- not implemented for now
   * @param other FeedforwardController object to assign from
   * @todo Implement
   */
  FeedforwardController& operator=(FeedforwardController&& other) = delete;

  /**
   * @brief Destructor
   */
  virtual ~FeedforwardController() = default;

  /**
   * @brief setController Assign control law
   * @param controllerTime
   * @param controllerFeedforward
   * @param controllerFeedback
   */
  void setController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward) {
    time_ = controllerTime;
    uff_ = controllerFeedforward;
  }

  /**
   * @brief reset revert back to empty controller
   */
  void reset() {
    linInterpolateUff_.reset();
    time_.clear();
    uff_.clear();
  }

  void clear() { reset(); }

  virtual input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    input_vector_t uff;
    linInterpolateUff_.interpolate(t, uff);
    return uff;
  }

  virtual void flatten(scalar_t time, scalar_array_t& flatArray) const override {
    input_vector_t uff;
    linInterpolateUff_.interpolate(time, uff);

    // TODO(jcarius) should we subtract k*x_ref here?
    flatArray = std::move(scalar_array_t(uff.data(), uff.data() + INPUT_DIM));
  }

  virtual void unFlatten(const scalar_array_t& timeArray, const std::vector<scalar_array_t const*>& flatArray2) override {
    time_ = timeArray;

    uff_.clear();
    uff_.reserve(flatArray2.size());

    if(flatArray2[0]->size() != INPUT_DIM){
      throw std::runtime_error("FeedforwardController::unFlatten received array of wrong length.");
    }

    for (const auto& arr : flatArray2) {  // loop through time
      uff_.emplace_back(Eigen::Map<const input_vector_t>(arr->data(), INPUT_DIM));
    }
  }

  /**
   * @brief Swap data with other object
   * @param other the object to be swapped with
   */
  virtual void swap(FeedforwardController<STATE_DIM, INPUT_DIM>& other) {
    using std::swap;  // enable ADL

    swap(time_, other.time_);
    swap(uff_, other.uff_);
  }

  /**
   * @brief Fills all the data containers with zeros. Does not change size, does not change time array.
   */
  void setZero() {
    std::fill(uff_.begin(), uff_.end(), input_vector_array_t::Zero());
  }

  /**
   * Returns whether the class is empty (i.e. whether its size is 0).
   *
   * @return true if the time container size is 0, false otherwise.
   */
  bool empty() const { return time_.empty(); }

  /**
   * Returns the size of the controller (in particular the time stamp).
   *
   * @return the size of the controller.
   */
  size_t size() const { return time_.size(); }

  virtual ControllerType getType() const override { return ControllerType::FEEDFORWARD; }

 public:
  scalar_array_t time_;
  input_vector_array_t uff_;

 protected:
  EigenLinearInterpolation<input_vector_t> linInterpolateUff_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(FeedforwardController<STATE_DIM, INPUT_DIM>& a, FeedforwardController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
