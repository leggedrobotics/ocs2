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

#include <vector>

namespace ocs2 {

/**
 * Computes one-dimensional distance transform for a sampled function based on lower envelope of parabolas method introduced by:
 * P. F. Felzenszwalb and D. P. Huttenlocher. "Distance transforms of sampled functions." (2012).
 *
 * @param numSamples: The size of the sampled function.
 * @param getValue: Lambda function to get value from with signature: ScalarType(size_t index).
 * @param setValue: Lambda function to set value to with signature: void(size_t index, ScalarType value).
 * @param start: The start index for iteration over.
 * @param end: The index of the element following the last element for iteration over.
 * @param vBuffer: A buffered memory of unsigned integers with size "numSamples". This memory holds the information
 *                 about the locations of parabolas in lower envelope.
 * @param zBuffer: A buffered memory of Scalar type with size "numSamples + 1". This memory holds the information
 *                 about the locations of boundaries between parabolas.
 *
 * @tparam GetValFunc: Template typename for inferring lambda expression with signature Scalar(size_t).
 * @tparam SetValFunc: Template typename for inferring lambda expression with signature void(size_t, Scalar).
 * @tparam Scalar: The Scalar type
 */
template <typename GetValFunc, typename SetValFunc, typename Scalar = float>
void computeDistanceTransform(size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, size_t start, size_t end,
                              std::vector<size_t>& vBuffer, std::vector<Scalar>& zBuffer);

/**
 * Computes one-dimensional distance transform for a sampled function based on lower envelope of parabolas method introduced by:
 * P. F. Felzenszwalb and D. P. Huttenlocher. "Distance transforms of sampled functions." (2012).
 *
 * @param numSamples: The size of the sampled function.
 * @param getValue: Lambda function to get value from with signature: ScalarType(size_t index).
 * @param setValue: Lambda function to set value to with signature: void(size_t index, ScalarType value).
 * @param setImageIndex: Lambda function to set mirror index with signature: void(size_t index, size_t mirrorIndex).
 * @param start: The start index for iteration over.
 * @param end: The index of the element following the last element for iteration over.
 * @param vBuffer: A buffered memory of unsigned integers with size "numSamples". This memory holds the information
 *                 about the locations of parabolas in lower envelope.
 * @param zBuffer: A buffered memory of Scalar type with size "numSamples + 1". This memory holds the information
 *                 about the locations of boundaries between parabolas.
 *
 * @tparam GetValFunc: Template typename for inferring lambda expression with signature Scalar(size_t).
 * @tparam SetValFunc: Template typename for inferring lambda expression with signature void(size_t, Scalar).
 * @tparam SetImageIndexFunc: Template typename for inferring lambda expression with signature void(size_t, size_t).
 * @tparam Scalar: The Scalar type
 */
template <typename GetValFunc, typename SetValFunc, typename SetImageIndexFunc, typename Scalar = float>
void computeDistanceTransform(size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, SetImageIndexFunc&& setImageIndex,
                              size_t start, size_t end, std::vector<size_t>& vBuffer, std::vector<Scalar>& zBuffer);

}  // namespace ocs2

#include "implementation/ComputeDistanceTransform.h"
