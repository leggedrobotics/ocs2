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
 * The linear interpolation of costInputStateDerivative, Pm, at index-alpha pair given modelDataTrajectory can
 * be computed as:
 *
 * ModelData::interpolate(indexAlpha, Pm, *modelDataTrajectory, ModelData::costInputStateDerivative);
 */

/*
 * Declares an access function of name FIELD such as time, dynamics, dynamicsBias, ...
 * For example the signature of function for dynamics is:
 * const dynamic_vector_t& dynamics(const ocs2::ModelDataBase::array_t* vec, size_t n) {
 *   return (*vec)[n].dynamic_;
 * }
 */
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD)                                                               \
  inline auto FIELD(const ocs2::ModelDataBase::array_t* vec, size_t ind)->const decltype((*vec)[ind].FIELD##_)& { \
    return (*vec)[ind].FIELD##_;                                                                                  \
  }

namespace ocs2 {
namespace ModelData {

using scalar_t = ocs2::ModelDataBase::scalar_t;
using dynamic_vector_t = ocs2::ModelDataBase::dynamic_vector_t;
using dynamic_matrix_t = ocs2::ModelDataBase::dynamic_matrix_t;

/**
 * Helper specialization of interpolate() of ModelData array types for scalar_t subfields.
 * Note that since partial specialization of function templates is not possible, it is not
 * possible to write a general interpolate() function with template argument Field_T.
 */
inline void interpolate(ocs2::LinearInterpolation::index_alpha_t indexAlpha, scalar_t& enquiryData,
                        const ocs2::ModelDataBase::array_t* dataPtr,
                        std::function<const scalar_t&(const ocs2::ModelDataBase::array_t*, size_t)> accessFun) {
  ocs2::LinearInterpolation::interpolate(indexAlpha, enquiryData, dataPtr, accessFun);
}

/**
 * Helper specialization of interpolate() of ModelData array types for dynamic_vector_t subfields.
 */
inline void interpolate(ocs2::LinearInterpolation::index_alpha_t indexAlpha, dynamic_vector_t& enquiryData,
                        const ocs2::ModelDataBase::array_t* dataPtr,
                        std::function<const dynamic_vector_t&(const ocs2::ModelDataBase::array_t*, size_t)> accessFun) {
  ocs2::LinearInterpolation::interpolate(indexAlpha, enquiryData, dataPtr, accessFun);
}

/**
 * Helper specialization of interpolate() of ModelData array types for scalar_t subfields.
 */
inline void interpolate(ocs2::LinearInterpolation::index_alpha_t indexAlpha, dynamic_matrix_t& enquiryData,
                        const ocs2::ModelDataBase::array_t* dataPtr,
                        std::function<const dynamic_matrix_t&(const ocs2::ModelDataBase::array_t*, size_t)> accessFun) {
  ocs2::LinearInterpolation::interpolate(indexAlpha, enquiryData, dataPtr, accessFun);
}

/**
 * Re-defining the timeSegment() in ModelData namespace.
 */
inline ocs2::LinearInterpolation::index_alpha_t timeSegment(ocs2::LinearInterpolation::scalar_t enquiryTime,
                                                            const std::vector<ocs2::LinearInterpolation::scalar_t>* timeArrayPtr) {
  return ocs2::LinearInterpolation::timeSegment(enquiryTime, timeArrayPtr);
}

/**
 * Access method for different subfields of the ModelData.
 */

// time
CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

// dynamics
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamics)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsBias)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsInputDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsCovariance)

// cost
CREATE_INTERPOLATION_ACCESS_FUNCTION(cost)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costStateSecondDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputSecondDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputStateDerivative)

// state equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateEqConstr)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateEqConstrStateDerivative)

// state-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstr)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstrStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstrInputDerivative)

}  // namespace ModelData
}  // namespace ocs2

#undef CREATE_INTERPOLATION_ACCESS_FUNCTION
