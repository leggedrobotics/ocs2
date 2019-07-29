#pragma once

#include "ocs2_core/control/ControllerBase.h"

#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * FeedforwardController provides a time-dependent control law without state-dependent feedback.
 * Commonly, this is used to wrap around a more general controller and extract only the feedforward portion.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class FeedforwardController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using float_array_t = typename Base::float_array_t;
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
  FeedforwardController() : Base(), linInterpolateUff_(&timeStamp_, &uffArray_) {}

  /**
   * @brief Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  FeedforwardController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward) : FeedforwardController() {
    setController(controllerTime, controllerFeedforward);
  }

  /**
   * @brief Constructor to initialize the feedforward input data with a general controller rolled-out along a nominal stateTrajectory
   * @param [in] controllerTime the times for the rollout
   * @param [in] stateTrajectory the states for the rollout
   * @param [in] controller the controller to extract the feedforward controls from during a rollout
   */
  FeedforwardController(const scalar_array_t& controllerTime, const state_vector_array_t& stateTrajectory, Base* controller)
      : FeedforwardController() {
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

  /**
   * @brief Copy constructor
   * @param other FeedforwardController object to copy from
   */
  FeedforwardController(const FeedforwardController& other) : FeedforwardController(other.timeStamp_, other.uffArray_) {}

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
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  void setController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward) {
    timeStamp_ = controllerTime;
    uffArray_ = controllerFeedforward;
  }

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    input_vector_t uff;
    linInterpolateUff_.interpolate(t, uff);
    return uff;
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
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

  void flattenSingle(scalar_t time, float_array_t& flatArray) const {
    input_vector_t uff;
    linInterpolateUff_.interpolate(time, uff);

    flatArray = std::move(float_array_t(uff.data(), uff.data() + INPUT_DIM));
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    if (flatArray2[0]->size() != INPUT_DIM) {
      throw std::runtime_error("FeedforwardController::unFlatten received array of wrong length.");
    }

    timeStamp_ = timeArray;

    uffArray_.clear();
    uffArray_.reserve(flatArray2.size());

    for (const auto& arr : flatArray2) {  // loop through time
      uffArray_.emplace_back(Eigen::Map<const Eigen::Matrix<float, INPUT_DIM, 1>>(arr->data(), INPUT_DIM).template cast<scalar_t>());
    }
  }

  ControllerType getType() const override { return ControllerType::FEEDFORWARD; }

  void clear() override {
    timeStamp_.clear();
    uffArray_.clear();
  }

  void setZero() override { std::fill(uffArray_.begin(), uffArray_.end(), input_vector_t::Zero()); }

  /**
   * Returns whether the class is empty (i.e. whether its size is 0).
   *
   * @return true if the time container size is 0, false otherwise.
   */
  bool empty() const override { return timeStamp_.empty(); }

  /**
   * @brief Swap data with other object.
   *
   * @param other the object to be swapped with
   */
  void swap(FeedforwardController<STATE_DIM, INPUT_DIM>& other) {
    using std::swap;  // enable ADL

    swap(timeStamp_, other.timeStamp_);
    swap(uffArray_, other.uffArray_);
  }

  FeedforwardController<STATE_DIM, INPUT_DIM> * clone() override {
    return new FeedforwardController<STATE_DIM, INPUT_DIM>(*this);
  }

  /**
   * Returns the size of the controller (in particular the time stamp).
   *
   * @return the size of the controller.
   */
  size_t size() const { return timeStamp_.size(); }

  void display() const override {
    for (int i=0;i<timeStamp_.size();i++) {
        std::cerr << "t\t" << timeStamp_[i] << "\tu\t" << uffArray_[i].transpose() << "\n";
      }
    std::cerr << std::endl;
  }

 public:
  scalar_array_t timeStamp_;
  input_vector_array_t uffArray_;

 protected:
  EigenLinearInterpolation<input_vector_t> linInterpolateUff_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(FeedforwardController<STATE_DIM, INPUT_DIM>& a, FeedforwardController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
