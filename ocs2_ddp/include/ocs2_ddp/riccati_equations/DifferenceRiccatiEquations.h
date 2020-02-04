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

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/model_data/ModelDataBase.h>

#include "ocs2_ddp/riccati_equations/RiccatiModificationBase.h"

namespace ocs2 {

/**
 * This class implements the Riccati difference equations for iLQR problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <int STATE_DIM, int INPUT_DIM>
class DifferenceRiccatiEquations {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

  /**
   * Default constructor.
   */
  DifferenceRiccatiEquations() = default;

  /**
   * Default destructor.
   */
  ~DifferenceRiccatiEquations() = default;

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] derivatives: d(allSs)/dz.
   */
  void computeMap(const ModelDataBase projectedModelData, const RiccatiModificationBase& riccatiModification, const state_matrix_t& SmNext,
                  const state_vector_t& SvNext, const scalar_t& sNext, dynamic_matrix_t& HmAugInvUUT_T_GmAug,
                  dynamic_vector_t& HmAugInvUUT_T_GvAug, state_matrix_t& Sm, state_vector_t& Sv, scalar_t& s) {
    const auto& HmAugInvUUT = riccatiModification.HmInverseConstrainedLowRank_;

    // precomputation
    Hm_ = projectedModelData.costInputSecondDerivative_;
    Bm_T_Sm_.noalias() = projectedModelData.dynamicsInputDerivative_.transpose() * SmNext;
    Hm_.noalias() += Bm_T_Sm_ * projectedModelData.dynamicsInputDerivative_;

    HmAugInvUUT_T_Hm_.noalias() = HmAugInvUUT.transpose() * Hm_;
    HmAugInvUUT_T_GvAug.noalias() = HmAugInvUUT_T_Hm_ * projectedModelData.stateInputEqConstr_;
    HmAugInvUUT_T_GmAug.noalias() = HmAugInvUUT_T_Hm_ * projectedModelData.stateInputEqConstrStateDerivative_;
    HmAugInvUUT_T_GmAug.noalias() += HmAugInvUUT.transpose() * riccatiModification.deltaPm_;
    HmAugInvUUT_T_Hm_HmAugInvUUT_.noalias() = HmAugInvUUT_T_Hm_ * HmAugInvUUT;

    // s + q
    s = sNext + projectedModelData.cost_;
    // Qv
    Sv = projectedModelData.costStateDerivative_;
    // Qm + deltaQm
    Sm = projectedModelData.costStateSecondDerivative_ + riccatiModification.deltaQm_;
    // Rv
    Gv_ = projectedModelData.costInputDerivative_;
    // Pm
    Gm_ = projectedModelData.costInputStateDerivative_;

    /*
     * Sm
     */
    // Am^T * Sm * Am
    Am_T_Sm_.noalias() = projectedModelData.dynamicsStateDerivative_.transpose() * SmNext;
    Sm.noalias() += Am_T_Sm_ * projectedModelData.dynamicsStateDerivative_;

    // Gm = Pm + Bm^T Sm Am
    Gm_.noalias() += projectedModelData.dynamicsInputDerivative_.transpose() * Am_T_Sm_.transpose();
    // Km = inv(HmAug) * Gm = (HmAugInvUUT * HmAugInvUUT^T) * Gm = HmAugInvUUT * HmAugInvUUT_T_Gm_
    HmAugInvUUT_T_Gm_.noalias() = HmAugInvUUT.transpose() * Gm_;
    HmAugInvUUT_T_GmAug += HmAugInvUUT_T_Gm_;

    // Km^T * Gm
    Km_T_Gm_.noalias() = -HmAugInvUUT_T_GmAug.transpose() * HmAugInvUUT_T_Gm_;

    // Km^T * Hm * Km
    HmAugInvUUT_T_Hm_Km_.noalias() = -HmAugInvUUT_T_Hm_HmAugInvUUT_.transpose() * HmAugInvUUT_T_GmAug;
    Km_T_Hm_Km_.noalias() = -HmAugInvUUT_T_Hm_Km_.transpose() * HmAugInvUUT_T_GmAug;

    // Sm
    Sm += Km_T_Gm_ + Km_T_Gm_.transpose() + Km_T_Hm_Km_;

    /*
     * Sv
     */
    // Sm * Hv
    Sm_Hv_.noalias() = SmNext * projectedModelData.dynamicsBias_;
    // Sv + Sm * Hv
    Sv_plus_Sm_Hv_ = SvNext + Sm_Hv_;
    // += Am^T * (Sv + Sm * Hv)
    Sv.noalias() += projectedModelData.dynamicsStateDerivative_.transpose() * Sv_plus_Sm_Hv_;

    // Gv = Rv + Bm^T (Sv + Sm * Hv)
    Gv_.noalias() += projectedModelData.dynamicsInputDerivative_.transpose() * Sv_plus_Sm_Hv_;
    HmAugInvUUT_T_Gv_.noalias() = HmAugInvUUT.transpose() * Gv_;
    HmAugInvUUT_T_GvAug += HmAugInvUUT_T_Gv_;

    // (Gm^T * Lv)
    Gm_T_Lv_.noalias() = -HmAugInvUUT_T_Gm_.transpose() * HmAugInvUUT_T_GvAug;

    // (Km^T * Gv)
    Km_T_Gv_.noalias() = -HmAugInvUUT_T_GmAug.transpose() * HmAugInvUUT_T_Gv_;

    // Km^T * Hm * Lv
    HmAugInvUUT_T_Hm_Lv_ = -HmAugInvUUT_T_Hm_HmAugInvUUT_.transpose() * HmAugInvUUT_T_GvAug;
    Km_T_Hm_Lv_ = -HmAugInvUUT_T_GmAug.transpose() * HmAugInvUUT_T_Hm_Lv_;

    // Sv
    Sv += Gm_T_Lv_ + Km_T_Gv_ + Km_T_Hm_Lv_;

    /*
     * s
     */
    // += Hv^T * (Sv + Sm * Hv)
    s += projectedModelData.dynamicsBias_.dot(Sv_plus_Sm_Hv_);
    // -= 0.5 Hv^T * Sm * Hv
    s -= 0.5 * projectedModelData.dynamicsBias_.dot(Sm_Hv_);
    // += Lv^T Gv
    s -= HmAugInvUUT_T_GvAug.dot(HmAugInvUUT_T_Gv_);
    // += Lv^T Hm Lv
    s -= 0.5 * HmAugInvUUT_T_GvAug.dot(HmAugInvUUT_T_Hm_Lv_);
  }

 private:
  dynamic_matrix_t Hm_;
  dynamic_matrix_t Bm_T_Sm_;
  dynamic_matrix_t Am_T_Sm_;
  dynamic_matrix_t HmAugInvUUT_T_Hm_;
  dynamic_matrix_t HmAugInvUUT_T_Hm_HmAugInvUUT_;

  dynamic_matrix_t Gm_;
  dynamic_matrix_t HmAugInvUUT_T_Gm_;
  dynamic_matrix_t Km_T_Gm_;
  dynamic_matrix_t HmAugInvUUT_T_Hm_Km_;
  dynamic_matrix_t Km_T_Hm_Km_;

  dynamic_vector_t Sm_Hv_;
  dynamic_vector_t Sv_plus_Sm_Hv_;

  dynamic_vector_t Gv_;
  dynamic_vector_t HmAugInvUUT_T_Gv_;
  dynamic_vector_t Gm_T_Lv_;
  dynamic_vector_t Km_T_Gv_;
  dynamic_vector_t HmAugInvUUT_T_Hm_Lv_;
  dynamic_vector_t Km_T_Hm_Lv_;
};

}  // namespace ocs2
