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
#include <Eigen/StdVector>

#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

namespace ocs2 {

/**
 * BVP sensitivity error equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityErrorEquations final : public OdeBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = OdeBase<STATE_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_matrix_array_t = typename DIMENSIONS::dynamic_matrix_array_t;

  /**
   * Constructor.
   */
  BvpSensitivityErrorEquations() = default;

  /**
   * Default destructor.
   */
  ~BvpSensitivityErrorEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>* clone() const {
    return new BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>(*this);
  }

  /**
   * Sets Data
   */
  void setData(const scalar_array_t* timeStampPtr, const ModelData::array_t* modelDataPtr, const ModelData::array_t* projectedModelDataPtr,
               const dynamic_matrix_array_t* RmInversePtr, const dynamic_matrix_array_t* RinvCholPtr,
               const input_vector_array_t* EvDevProjectedPtr, const scalar_array_t* SmTimeStampPtr, const state_matrix_array_t* SmPtr) {
    timeStampPtr_ = timeStampPtr;
    modelDataPtr_ = modelDataPtr;
    projectedModelDataPtr_ = projectedModelDataPtr;
    RmInversePtr_ = RmInversePtr;
    RinvCholPtr_ = RinvCholPtr;
    EvDevProjectedPtr_ = EvDevProjectedPtr;
    SmTimeStampPtr_ = SmTimeStampPtr;
    SmPtr_ = SmPtr;
  }

  /**
   * Computes Derivative
   * @param [in] time: Normalized transition time
   * @param [in] Mv: transition state
   * @param [out] dMv: mapped state after transition
   */
  void computeFlowMap(const scalar_t& z, const state_vector_t& Mve, state_vector_t& dMvedz) override {
    // denormalized time
    const scalar_t t = -z;
    auto indexAlpha = LinearInterpolation::timeSegment(t, timeStampPtr_);

    ModelData::interpolate(indexAlpha, Bm_, modelDataPtr_, ModelData::dynamicsInputDerivative);
    ModelData::interpolate(indexAlpha, Rm_, modelDataPtr_, ModelData::costInputSecondDerivative);
    ModelData::interpolate(indexAlpha, Pm_, modelDataPtr_, ModelData::costInputStateDerivative);

    ModelData::interpolate(indexAlpha, AmConstrained_, projectedModelDataPtr_, ModelData::dynamicsStateDerivative);
    ModelData::interpolate(indexAlpha, CmProjected_, projectedModelDataPtr_, ModelData::stateInputEqConstrStateDerivative);

    LinearInterpolation::interpolate(indexAlpha, RmInverse_, RmInversePtr_);
    LinearInterpolation::interpolate(indexAlpha, RinvChol_, RinvCholPtr_);
    LinearInterpolation::interpolate(indexAlpha, EvDevProjected_, EvDevProjectedPtr_);

    LinearInterpolation::interpolate(t, Sm_, SmTimeStampPtr_, SmPtr_);

    // Lm
    // TODO: Double check if equations are correct after change to cholesky decomposition approach
    Lm_ = RinvChol_.transpose() * (Pm_ + Bm_.transpose() * Sm_);

    dMvedz =
        (AmConstrained_ - Bm_ * RinvChol_ * Lm_).transpose() * Mve + (CmProjected_ - RinvChol_ * Lm_).transpose() * Rm_ * EvDevProjected_;
  }

 private:
  scalar_t multiplier_ = 0.0;

  const scalar_array_t* timeStampPtr_;
  const ModelData::array_t* modelDataPtr_;
  const ModelData::array_t* projectedModelDataPtr_;
  const dynamic_matrix_array_t* RmInversePtr_;
  const dynamic_matrix_array_t* RinvCholPtr_;
  const input_vector_array_t* EvDevProjectedPtr_;
  const scalar_array_t* SmTimeStampPtr_;
  const state_matrix_array_t* SmPtr_;

  dynamic_matrix_t Bm_;
  dynamic_matrix_t Rm_;
  dynamic_matrix_t Pm_;
  dynamic_matrix_t AmConstrained_;
  dynamic_matrix_t CmProjected_;
  dynamic_matrix_t RmInverse_;
  dynamic_matrix_t RinvChol_;
  input_vector_t EvDevProjected_;
  state_matrix_t Sm_;
  dynamic_matrix_t Lm_;
};

}  // namespace ocs2
