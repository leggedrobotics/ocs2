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

#include <limits>
#include <utility>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename GetValFunc, typename SetValFunc, typename Scalar>
void computeDistanceTransform(size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, size_t start, size_t end,
                              std::vector<size_t>& vBuffer, std::vector<Scalar>& zBuffer) {
  computeDistanceTransform(
      numSamples, std::forward<GetValFunc>(getValue), std::forward<SetValFunc>(setValue), [&](size_t x, size_t ind) {}, start, end, vBuffer,
      zBuffer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename GetValFunc, typename SetValFunc, typename SetImageIndexFunc, typename Scalar>
void computeDistanceTransform(size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, SetImageIndexFunc&& setImageIndex,
                              size_t start, size_t end, std::vector<size_t>& vBuffer, std::vector<Scalar>& zBuffer) {
  constexpr auto PlusInfinity = std::numeric_limits<Scalar>::max();
  constexpr auto MinusInfinity = std::numeric_limits<Scalar>::lowest();

  // make sure the buffer memories has the right size
  if (vBuffer.size() < numSamples) {
    vBuffer.resize(numSamples);  // locations of parabolas in lower envelope
  }
  if (zBuffer.size() < numSamples + 1) {
    zBuffer.resize(numSamples + 1);  // locations of boundaries between parabolas
  }

  // initialization
  vBuffer[start] = start;
  zBuffer[start] = MinusInfinity;
  zBuffer[start + 1] = PlusInfinity;
  size_t k = start;  // index of rightmost parabola in lower envelope

  // compute lower envelope
  for (size_t q = start + 1; q < end; q++) {
    k++;  // compensates for first line of next do-while block

    Scalar s = 0;
    do {
      k--;
      // compute horizontal position of intersection between the parabola from q & the current lowest parabola
      s = ((getValue(q) + q * q) - (getValue(vBuffer[k]) + vBuffer[k] * vBuffer[k])) / (2 * q - 2 * vBuffer[k]);
    } while (s <= zBuffer[k]);

    k++;
    vBuffer[k] = q;
    zBuffer[k] = s;
    zBuffer[k + 1] = PlusInfinity;
  }  // end of for loop

  // fill in values of distance transform
  k = start;
  for (size_t q = start; q < end; q++) {
    while (zBuffer[k + 1] < q) {
      k++;
    }
    const size_t ind = vBuffer[k];
    const Scalar val = (q - ind) * (q - ind) + getValue(ind);
    setValue(q, val);
    setImageIndex(q, ind);
  }  // end of for loop
}

}  // namespace ocs2
