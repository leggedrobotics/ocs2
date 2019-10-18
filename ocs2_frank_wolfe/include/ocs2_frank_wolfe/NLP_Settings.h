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

#include <iostream>

namespace ocs2 {

/**
 * This structure contains the settings for the gradient-descent algorithm.
 */
class NLP_Settings {
 public:
  NLP_Settings()
      : displayInfo_(true),
        maxIterations_(1000),
        minRelCost_(1e-6),
        maxLearningRate_(1.0),
        minLearningRate_(0.05),
        useAscendingLineSearchNLP_(true) {}

  /** This value determines to display the log output.*/
  bool displayInfo_;
  /** This value determines the maximum number of algorithm iterations.*/
  size_t maxIterations_;
  /** This value determines the termination condition based on the minimum relative changes of the cost.*/
  double minRelCost_;
  /** This value determines the maximum step size for the line search scheme.*/
  double maxLearningRate_;
  /** This value determines the minimum step size for the line search scheme.*/
  double minLearningRate_;
  /**
   * This value determines the line search scheme to be used. \n
   * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
   * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
   * */
  bool useAscendingLineSearchNLP_;
};

}  // namespace ocs2
