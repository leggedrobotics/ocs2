#pragma once

#include "ocs2_core/control/Controller.h"

#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {


template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearController : public Controller<STATE_DIM, INPUT_DIM>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dimensions_t = Dimensions<STATE_DIM,INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

  LinearController() = default;

  LinearController(
                     const scalar_array_t& controllerTime,
                     const input_vector_array_t& controllerFeedforward,
                     const input_state_matrix_array_t& controllerFeedback)
    {setController(controllerTime, controllerFeedforward, controllerFeedback);}

  virtual ~LinearController() = default;

  void setController(
                          const scalar_array_t& controllerTime,
                          const input_vector_array_t& controllerFeedforward,
                          const input_state_matrix_array_t& controllerFeedback) {

    time_ = controllerTime;
    uff_ = controllerFeedforward;
    k_ = controllerFeedback;

                  linInterpolateUff_.setTimeStamp(&time_);
                  linInterpolateUff_.setData(&uff_);

                  linInterpolateK_.setTimeStamp(&time_);
                  linInterpolateK_.setData(&k_);
  }

  void reset() {
    linInterpolateUff_.reset();
    linInterpolateK_.reset();
    time_.clear();
    uff_.clear();
    k_.clear();
  }

  /**
   * Computes the control command at a given time and state.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current input.
   */
  virtual input_vector_t computeInput(
                  const scalar_t& t,
      const state_vector_t& x) override {
    input_vector_t uff;
    linInterpolateUff_.interpolate(t, uff);
    auto greatestLessTimeStampIndex = linInterpolateUff_.getGreatestLessTimeStampIndex();

    input_state_matrix_t k;
    linInterpolateK_.interpolate(t, k, greatestLessTimeStampIndex);

    return uff + k*x;
  }


public:
  scalar_array_t time_;
  input_vector_array_t uff_;
  input_state_matrix_array_t k_;

  protected:
      EigenLinearInterpolation<input_vector_t> linInterpolateUff_;
      EigenLinearInterpolation<input_state_matrix_t> linInterpolateK_;



};


}
