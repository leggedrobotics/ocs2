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

#include <ocs2_ddp/GLQP.h>

#include "ocs2_ocs2/GDDP.h"

namespace ocs2 {

/**
 * GSLQ Solver Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 * @tparam OUTPUT_DIM
 * @tparam NUM_Subsystems
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
class GSLQSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef GLQP<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GLQP_t;
  typedef GSLQ<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GSLQ_t;

  typedef Dimensions<STATE_DIM, INPUT_DIM, OUTPUT_DIM> DIMENSIONS;
  typedef typename DIMENSIONS::controller_t controller_t;
  typedef typename DIMENSIONS::Options Options_t;
  typedef typename DIMENSIONS::MP_Options MP_Options_t;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
  typedef typename DIMENSIONS::input_state_t input_state_t;
  typedef typename DIMENSIONS::input_state_array_t input_state_array_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
  typedef typename DIMENSIONS::state_input_matrix_t state_input_matrix_t;
  typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

  typedef Eigen::Matrix<double, NUM_Subsystems - 1, 1> parameters_t;

  /**
   * Constructor
   */
  GSLQSolver(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
             const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
             const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
             const state_vector_array_t& stateOperatingPoints, const input_vector_array_t& inputOperatingPoints,
             const std::vector<size_t>& systemStockIndex, const Options_t& options = Options_t::Options(),
             const MP_Options_t& mpOptions = MP_Options_t::MP_Options())

      : subsystemDynamicsPtr_(subsystemDynamicsPtr),
        subsystemDerivativesPtr_(subsystemDerivativesPtr),
        subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr),
        stateOperatingPoints_(stateOperatingPoints),
        inputOperatingPoints_(inputOperatingPoints),
        systemStockIndex_(systemStockIndex),
        options_(options),
        mp_options_(mpOptions),
        controllersStock_(NUM_Subsystems) {}

  virtual ~GSLQSolver() {}

  /**
   * Returns cost
   * @return scalar_t:
   */
  scalar_t cost() { return cost_; }

  parameters_t costDerivative() { return costDerivative_; }

  /**
   * Returns controllerstock
   * @param [out] controllersStock
   */
  void controller(std::vector<controller_t>& controllersStock) { controllersStock = controllersStock_; }

  /**
   * Reset function
   */
  void reset() {
    controllersStockBag_.clear();
    parametersBag_.clear();
  }

  /**
   * Returns the options
   * @return Options_t:
   */
  Options_t& options() { return options_; }

  /**
   * Returns the mp_options
   * @return MP_Options_t:
   */
  MP_Options_t& mp_options() { return mp_options_; }

  /**
   * Run function
   * @param [in] initTime
   * @param [in] initState
   * @param [in] switchingTimes
   */
  void run(const double& initTime, const state_vector_t& initState, const std::vector<scalar_t>& switchingTimes);

 protected:
  size_t findNearestController(const Eigen::Matrix<double, NUM_Subsystems - 1, 1>& parameters) const;

 private:
  std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtr_;
  std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtr_;
  std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtr_;

  state_vector_array_t stateOperatingPoints_;
  input_vector_array_t inputOperatingPoints_;

  std::vector<size_t> systemStockIndex_;

  Options_t options_;
  MP_Options_t mp_options_;

  std::vector<parameters_t, Eigen::aligned_allocator<parameters_t> > parametersBag_;
  std::vector<std::vector<controller_t> > controllersStockBag_;

  scalar_t cost_;
  parameters_t costDerivative_;
  std::vector<controller_t> controllersStock_;
};

}  // namespace ocs2

#include "implementation/GSLQSolver.h"
