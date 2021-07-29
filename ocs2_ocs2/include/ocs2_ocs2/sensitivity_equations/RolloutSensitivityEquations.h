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

#include <Eigen/Dense>

#include <array>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

namespace ocs2 {

/**
 * Rollout sensitivity equations class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RolloutSensitivityEquations final : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ControlledSystemBase<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;

  /**
   * The default constructor.
   */
  RolloutSensitivityEquations() = default;

  /**
   * Default destructor.
   */
  ~RolloutSensitivityEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>* clone() const override {
    return new RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>(*this);
  }

  /**
   * Sets Data
   */
  void setData(const scalar_array_t* timeTrajectoryPtr, const ModelData::array_t* modelDataPtr,
               const scalar_array_t* sensitivityControllerTimePtr, const input_vector_array_t* sensitivityControllerFeedforwardPtr,
               const input_state_matrix_array_t* sensitivityControllerFeedbackPtr) {
    timeTrajectoryPtr_ = timeTrajectoryPtr;
    modelDataPtr_ = modelDataPtr;
    linearController_.setController(*sensitivityControllerTimePtr, *sensitivityControllerFeedforwardPtr, *sensitivityControllerFeedbackPtr);
    this->setController(&linearController_);
  }

  /**
   * Sets the multiplier of exogenous part of the equation. It is either zero
   * or plus-minus 1/(s_{i+1}-s_{i})
   *
   * @param [in] multiplier: the multiplier of exogenous part of the equation.
   */
  void setMultiplier(const scalar_t& multiplier) { multiplier_ = multiplier; }

  /**
   * Computes Derivative
   *
   * @param [in] t: time
   * @param [in] nabla_Xv: state sensitivity vector
   * @param [in] nabla_Uv: input sensitivity vector which is computed by using sensitivity controller.
   * @param [out] derivatives: time derivative of the state sensitivity vector
   */
  void computeFlowMap(const scalar_t& t, const state_vector_t& nabla_x, const input_vector_t& nabla_u,
                      state_vector_t& derivative) override {
    const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectoryPtr_);

    ModelData::interpolate(indexAlpha, Am_, modelDataPtr_, ModelData::dynamicsStateDerivative);
    ModelData::interpolate(indexAlpha, Bm_, modelDataPtr_, ModelData::dynamicsInputDerivative);

    if (!numerics::almost_eq(multiplier_, 0.0)) {
      ModelData::interpolate(indexAlpha, Fv_, modelDataPtr_, ModelData::dynamics);
      derivative = Am_ * nabla_x + Bm_ * nabla_u + multiplier_ * Fv_;
    } else {
      derivative = Am_ * nabla_x + Bm_ * nabla_u;
    }
  }

 protected:
  scalar_t multiplier_ = 0.0;
  const scalar_array_t* timeTrajectoryPtr_;
  const ModelData::array_t* modelDataPtr_;
  linear_controller_t linearController_;

  dynamic_vector_t Fv_;
  dynamic_matrix_t Am_;
  dynamic_matrix_t Bm_;
};

}  // namespace ocs2
