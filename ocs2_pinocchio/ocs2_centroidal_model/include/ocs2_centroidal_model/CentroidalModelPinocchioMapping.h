/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_centroidal_model/CentroidalModelPinocchioInterface.h"

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {

template <typename SCALAR>
class CentroidalModelPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using typename PinocchioStateInputMapping<SCALAR>::vector_t;
  using typename PinocchioStateInputMapping<SCALAR>::matrix_t;

  CentroidalModelPinocchioMapping(size_t stateDim, size_t inputDim,
                                  const CentroidalModelPinocchioInterface<SCALAR>& centroidalModelPinocchioInterface);

  ~CentroidalModelPinocchioMapping() override = default;

  CentroidalModelPinocchioMapping<SCALAR>* clone() const override { return new CentroidalModelPinocchioMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const;

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

 private:
  mutable CentroidalModelPinocchioInterface<SCALAR> centroidalModelPinocchioInterface_;
  const size_t stateDim_;
  const size_t inputDim_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class CentroidalModelPinocchioMapping<scalar_t>;
extern template class CentroidalModelPinocchioMapping<ad_scalar_t>;
}  // namespace ocs2
