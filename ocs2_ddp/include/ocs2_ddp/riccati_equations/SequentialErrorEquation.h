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

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * This class implements the time-normalized Error equation of Riccati equation.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialErrorEquation final : public OdeBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = OdeBase<STATE_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;

  /**
   * Default constructor.
   */
  SequentialErrorEquation() = default;

  /**
   * Default destructor.
   */
  ~SequentialErrorEquation() = default;

  /**
   * Sets coefficients of the model.
   *
   * @param [in] switchingTimeStart: The start time of the subsystem.
   * @param [in] switchingTimeFinal: The final time of the subsystem.
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
   * @param [in] GmPtr: A pointer to the trajectory of \f$ G_m(t) \f$ .
   */
  void setData(const scalar_array_t* timeStampPtr, const state_vector_array_t* GvPtr, const state_matrix_array_t* GmPtr) {
    BASE::resetNumFunctionCalls();
    timeStampPtr_ = timeStampPtr;
    GvPtr_ = GvPtr;
    GmPtr_ = GmPtr;
  }

  /**
   * Error Riccati jump map at switching moments
   *
   * @param [in] time: Normalized transition time
   * @param [in] state: transition state
   * @param [out] mappedState: mapped state after transition
   */
  void computeJumpMap(const scalar_t& z, const state_vector_t& state, state_vector_t& mappedState) override { mappedState = state; }

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] Sve: Current Sve.
   * @param [out] derivatives: d(Sve)/dz
   */
  void computeFlowMap(const scalar_t& z, const state_vector_t& Sve, state_vector_t& derivatives) {
    BASE::numFunctionCalls_++;

    // normal time
    const scalar_t t = -z;

    const auto indexAlpha = EigenLinearInterpolation<state_matrix_t>::interpolate(t, Gm_, timeStampPtr_, GmPtr_);

    // derivatives = Gv + Gm*Sve
    EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, derivatives, GvPtr_);
    derivatives.noalias() += Gm_.transpose() * Sve;
  }

 private:
  const scalar_array_t* timeStampPtr_;
  const state_vector_array_t* GvPtr_;
  const state_matrix_array_t* GmPtr_;

  // members required in computeFlowMap
  state_matrix_t Gm_;
};

}  // namespace ocs2
