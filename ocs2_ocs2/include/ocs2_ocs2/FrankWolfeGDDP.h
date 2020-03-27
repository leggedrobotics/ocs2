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

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>

#include <ocs2_frank_wolfe/FrankWolfeDescentDirection.h>
#include <ocs2_frank_wolfe/NLP_Constraints.h>

#include "ocs2_ocs2/GDDP.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class FrankWolfeGDDP : public GDDP<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef GDDP<STATE_DIM, INPUT_DIM> BASE;

  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::slq_data_collector_t;

  /**
   * Default constructor.
   */
  FrankWolfeGDDP() : FrankWolfeGDDP(GDDP_Settings()) {}

  /**
   * Constructor.
   *
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   */
  explicit FrankWolfeGDDP(const GDDP_Settings& gddpSettings);

  /**
   * Default destructor.
   */
  ~FrankWolfeGDDP() = default;

  /**
   * Runs the GDDP to compute the gradient of the cost function w.r.t. the event times while respecting
   * the provided constraints.
   *
   * @param [in] dcPtr: A constant pointer to SLQ data collector which already collected the SLQ variables.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse. Note that if there is no
   * limit for a direction set associated element to zero.
   * @param [in] eventTimeConstraintPtr: A pointer to the NLP constraints for event times.
   */
  void run(const slq_data_collector_t* dcPtr, const dynamic_vector_t& maxGradientInverse, NLP_Constraints* eventTimeConstraintPtr);

 protected:
  /***********
   * Variables
   **********/
  std::unique_ptr<FrankWolfeDescentDirection> frankWolfeDescentDirectionPtr_;
};

}  // namespace ocs2

#include "implementation/FrankWolfeGDDP.h"
