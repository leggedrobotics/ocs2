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

#include <ocs2_oc/rollout/RolloutBase.h>

namespace ocs2 {

/**
 * This rollout class uses a physics simulator backend for integrating the system dynamics
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SimulatorRollout : public RolloutBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = RolloutBase<STATE_DIM, INPUT_DIM>;

  using controller_t = typename Base::controller_t;
  using size_array_t = typename Base::size_array_t;
  using scalar_t = typename Base::scalar_t;
  using scalar_array_t = typename Base::scalar_array_t;
  using state_vector_t = typename Base::state_vector_t;
  using state_vector_array_t = typename Base::state_vector_array_t;
  using input_vector_t = typename Base::input_vector_t;
  using input_vector_array_t = typename Base::input_vector_array_t;

  using logic_rules_machine_t = HybridLogicRulesMachine;

  SimulatorRollout(std::string pathToUrdf, Rollout_Settings rolloutSettings = Rollout_Settings(), char algorithmName[] = nullptr)
      : Base(std::move(rolloutSettings), std::move(algorithmName)) {
    /** @todo(Martin)
     * Initialize the class here. Add further arguments to the constructor if needed
     * You should check that the state and input dimension (template arguments) matches the URDF model (throw exception if not)
     */
  }

  state_vector_t run(size_t partitionIndex, scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                     controller_t* controller, logic_rules_machine_t& logicRulesMachine, scalar_array_t& timeTrajectory,
                     size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                     input_vector_array_t& inputTrajectory) override {
    /** @todo(Martin) Implementation of the actual simulator step-physics here
     * Relevant inputs: initTime, initState, finalTime, controller
     * Outputs to be filled: timeTrajectory, stateTrajectory, inputTrajectory
     */
  }

 protected:
  //! @todo(Martin) Any member variables you might need
};

}  // namespace ocs2
