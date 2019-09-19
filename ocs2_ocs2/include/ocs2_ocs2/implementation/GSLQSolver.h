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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
size_t GSLQSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::findNearestController(
    const Eigen::Matrix<double, NUM_Subsystems - 1, 1>& enquiry) const {
  if (parametersBag_.size() == 0) throw std::runtime_error("controllerStock bag is empty.");

  // evaluating distance
  std::vector<double> distance(parametersBag_.size());
  for (size_t i = 0; i < parametersBag_.size(); i++) distance[i] = (parametersBag_[i] - enquiry).squaredNorm();

  // min index
  auto it = std::min_element(distance.begin(), distance.end());
  size_t index = std::distance(distance.begin(), it);

  return index;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
void GSLQSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::run(const double& initTime, const state_vector_t& initState,
                                                                       const std::vector<scalar_t>& switchingTimes) {
  // defining the parameter vector which is the switching times
  Eigen::Matrix<double, NUM_Subsystems - 1, 1> parameters = Eigen::VectorXd::Map(switchingTimes.data() + 1, NUM_Subsystems - 1);

  std::vector<controller_t> controllersStock(NUM_Subsystems);
  if (parametersBag_.size() == 0 || options_.warmStartGSLQ_ == false) {
    // GLQP initialization
    GLQP_t glqp(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, stateOperatingPoints_, inputOperatingPoints_,
                systemStockIndex_);

    glqp.run(switchingTimes);

    // GLQP controller
    glqp.getController(controllersStock);
  } else {
    // find nearest controller
    size_t index = findNearestController(parameters);
    controllersStock = controllersStockBag_.at(index);
  }

  // GSLQ
  GSLQ_t gslq(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, controllersStock, systemStockIndex_, options_);
  gslq.run(initTime, initState, switchingTimes);

  // cost funtion
  gslq.getValueFuntion(0.0, initState, cost_);

  // cost funtion jacobian
  gslq.getCostFuntionDerivative(costDerivative_);

  // GSLQ controller
  gslq.getController(controllersStock_);

  // saving to bag
  parametersBag_.push_back(parameters);
  controllersStockBag_.push_back(controllersStock_);
}

}  // namespace ocs2
