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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
FrankWolfeGDDP<STATE_DIM, INPUT_DIM>::FrankWolfeGDDP(const GDDP_Settings& gddpSettings)
    : BASE(gddpSettings), frankWolfeDescentDirectionPtr_(new FrankWolfeDescentDirection(gddpSettings.displayInfo_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void FrankWolfeGDDP<STATE_DIM, INPUT_DIM>::run(const slq_data_collector_t* dcPtr, const dynamic_vector_t& maxGradientInverse,
                                               NLP_Constraints* eventTimeConstraintPtr) {
  // run the base method which will calculate the gradient
  this->run(dcPtr);

  // no need for using Frank-Wolfe if there is no constraint
  if (!eventTimeConstraintPtr) {
    return;
  }

  // display
  if (this->gddpSettings_.displayInfo_) {
    std::cerr << "Gradient:             " << this->nominalCostFuntionDerivative_.transpose() << std::endl;
  }

  // compute the projected gradient
  dynamic_vector_t fwDescentDirection;
  frankWolfeDescentDirectionPtr_->run(this->eventTimes_, this->nominalCostFuntionDerivative_, maxGradientInverse, eventTimeConstraintPtr,
                                      fwDescentDirection);
  // since it is gradient not the descent direction
  this->nominalCostFuntionDerivative_ = -fwDescentDirection;

  // display
  if (this->gddpSettings_.displayInfo_) {
    std::cerr << "Frank-Wolfe gradient: " << this->nominalCostFuntionDerivative_.transpose() << std::endl;
  }
}

}  // namespace ocs2
