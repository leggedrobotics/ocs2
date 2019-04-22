#pragma once

#include "ocs2_core/control/Controller.h"

#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * LinearController implements a time and state dependent controller of the
 * form u[x,t] = k[t] * x + uff[t]
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearController : public Controller<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Controller<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
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
  LinearController() : Base(), linInterpolateUff_(&time_, &uff_), linInterpolateK_(&time_, &k_) {}

  /**
   * @brief Constructor initializes all required members of the controller
   */
  LinearController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward,
                   const input_state_matrix_array_t& controllerFeedback)
      : LinearController() {
    setController(controllerTime, controllerFeedforward, controllerFeedback);
  }

  /**
   * @brief Copy constructor
   * @param other LinearController object to copy from
   */
  LinearController(const LinearController& other) : LinearController(other.time_, other.uff_, other.k_) { deltaUff_ = other.deltaUff_; }

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
  virtual ~LinearController() = default;

  /**
   * @brief setController Assign control law
   * @param controllerTime
   * @param controllerFeedforward
   * @param controllerFeedback
   */
  void setController(const scalar_array_t& controllerTime, const input_vector_array_t& controllerFeedforward,
                     const input_state_matrix_array_t& controllerFeedback) {
    time_ = controllerTime;
    uff_ = controllerFeedforward;
    k_ = controllerFeedback;
  }

  /**
   * @brief reset revert back to empty controller
   */
  void reset() {
    linInterpolateUff_.reset();
    linInterpolateK_.reset();
    time_.clear();
    uff_.clear();
    k_.clear();
  }

  void clear() { reset(); }

  virtual input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    input_vector_t uff;
    linInterpolateUff_.interpolate(t, uff);
    auto greatestLessTimeStampIndex = linInterpolateUff_.getGreatestLessTimeStampIndex();

    input_state_matrix_t k;
    linInterpolateK_.interpolate(t, k, greatestLessTimeStampIndex);

    return uff + k * x;
  }

  virtual void flatten(scalar_t time, scalar_array_t& flatArray) const override {
    flatArray.clear();
    flatArray.resize(INPUT_DIM + INPUT_DIM * STATE_DIM);

    input_vector_t uff;
    linInterpolateUff_.interpolate(time, uff);
    const auto greatestLessTimeStampIndex = linInterpolateUff_.getGreatestLessTimeStampIndex();

    input_state_matrix_t k;
    linInterpolateK_.interpolate(time, k, greatestLessTimeStampIndex);

    for (int i = 0; i < INPUT_DIM; i++) {  // i loops through input dim
      flatArray[i * (STATE_DIM + 1) + 0] = uff(i);
      for (int j = 0; j < STATE_DIM; j++) {  // j loops through state dim
        flatArray[i * (STATE_DIM + 1) + j + 1] = k(i, j);
      }
    }
  }

  virtual void unFlatten(const scalar_array_t& timeArray, const std::vector<scalar_array_t const*>& flatArray2) override {
    time_ = timeArray;

    uff_.clear();
    uff_.reserve(flatArray2.size());
    k_.clear();
    k_.reserve(flatArray2.size());

    if(flatArray2[0]->size() != INPUT_DIM + INPUT_DIM * STATE_DIM){
      throw std::runtime_error("LinearController::unFlatten received array of wrong length.");
    }

    for (const auto& arr : flatArray2) {  // loop through time
      uff_.emplace_back(input_vector_t::Zero());
      k_.emplace_back(input_state_matrix_t::Zero());

      for (int i = 0; i < INPUT_DIM; i++) {  // loop through input dim
        uff_.back()(i) = (*arr)[i * (STATE_DIM + 1) + 0];
        k_.back().row(i) = Eigen::Map<const Eigen::Matrix<scalar_t, 1, STATE_DIM>>(&((*arr)[i * (STATE_DIM + 1) + 1]), STATE_DIM);
      }
    }
  }

  /**
   * @brief Swap data with other object
   * @param other the object to be swapped with
   */
  virtual void swap(LinearController<STATE_DIM, INPUT_DIM>& other) {
    using std::swap;  // enable ADL

    swap(time_, other.time_);
    swap(uff_, other.uff_);
    swap(deltaUff_, other.deltaUff_);
    swap(k_, other.k_);
  }

  /**
   * @brief Fills all the data containers with zeros. Does not change size, does not change time array.
   */
  void setZero() {
    std::fill(uff_.begin(), uff_.end(), input_vector_array_t::Zero());
    std::fill(deltaUff_.begin(), deltaUff_.end(), input_vector_array_t::Zero());
    std::fill(k_.begin(), k_.end(), input_state_matrix_array_t::Zero());
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

  virtual ControllerType getType() const override { return ControllerType::LINEAR; }

 public:
  scalar_array_t time_;
  input_vector_array_t uff_;
  input_vector_array_t deltaUff_;
  input_state_matrix_array_t k_;

 protected:
  EigenLinearInterpolation<input_vector_t> linInterpolateUff_;
  EigenLinearInterpolation<input_state_matrix_t> linInterpolateK_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
void swap(LinearController<STATE_DIM, INPUT_DIM>& a, LinearController<STATE_DIM, INPUT_DIM>& b) {
  a.swap(b);
}

}  // namespace ocs2
