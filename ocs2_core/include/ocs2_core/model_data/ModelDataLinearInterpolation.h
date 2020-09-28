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

#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/model_data/ModelDataBase.h"

/*
 * @file
 * The linear interpolation of cost inpute-state derivative, Pm, at index-alpha pair given modelDataTrajectory can
 * be computed as:
 *
 * ModelData::interpolate<ModelData::cost_dfdux>(indexAlpha, Pm, &modelDataTrajectory);
 */

/*
 * Declares an access function class of name FIELD such as time, dynamics, dynamicsBias, ...
 * For example the signature of function for dynamics is:
 * struct dynamics {
 *   const vector_t& operator()(const ModelDataBase& m) { return m.dynamics_; }
 * };
 */
#define CREATE_INTERPOLATION_ACCESS_FUNCTOR(FIELD)                                                \
  struct FIELD {                                                                                  \
    auto operator()(const ModelDataBase& m) -> const decltype(m.FIELD##_)& { return m.FIELD##_; } \
  };

#define CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(FIELD, SUBFIELD)                                               \
  struct FIELD##_##SUBFIELD {                                                                                       \
    auto operator()(const ModelDataBase& m) -> const decltype(m.FIELD##_.SUBFIELD)& { return m.FIELD##_.SUBFIELD; } \
  };

namespace ocs2 {
namespace ModelData {

/**
 * Helper specialization of interpolate() of ModelData array types.
 */
template <class AccesFun, typename Field>
void interpolate(ocs2::LinearInterpolation::index_alpha_t indexAlpha, Field& enquiryData, const std::vector<ocs2::ModelDataBase>* dataPtr) {
  ocs2::LinearInterpolation::interpolateField<AccesFun>(indexAlpha, enquiryData, dataPtr);
}

/**
 * Re-defining the timeSegment() in ModelData namespace.
 */
inline ocs2::LinearInterpolation::index_alpha_t timeSegment(scalar_t enquiryTime, const scalar_array_t* timeArrayPtr) {
  return ocs2::LinearInterpolation::timeSegment(enquiryTime, timeArrayPtr);
}

/**
 * Access method for different subfields of the ModelData.
 */

// time
CREATE_INTERPOLATION_ACCESS_FUNCTOR(time)

// dynamics
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR(dynamicsBias)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR(dynamicsCovariance)

// cost
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdxx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfduu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdux)

// state equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateEqConstr, dfdx)

// state-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, dfdu)

}  // namespace ModelData
}  // namespace ocs2

#undef CREATE_INTERPOLATION_ACCESS_FUNCTOR
#undef CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD
