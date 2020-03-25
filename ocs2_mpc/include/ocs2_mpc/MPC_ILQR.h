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

#include <ocs2_core/Dimensions.h>
#include <ocs2_ddp/DDP_BASE.h>
#include <ocs2_ddp/ILQR.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2 {

/**
 * This an MPC implementation with ILQR optimal control solver.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MPC_ILQR : public MPC_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = MPC_BASE<STATE_DIM, INPUT_DIM>;

  using typename BASE::DIMENSIONS;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using scalar_array2_t = typename DIMENSIONS::scalar_array2_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using size_array2_t = typename DIMENSIONS::size_array2_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename DIMENSIONS::input_state_matrix_array2_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;

  using typename BASE::controller_const_ptr_array_t;
  using typename BASE::controller_ptr_array_t;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using linear_controller_array_t = typename linear_controller_t::array_t;

  using ddp_base_t = ocs2::DDP_BASE<STATE_DIM, INPUT_DIM>;

  using ilqr_t = ocs2::ILQR<STATE_DIM, INPUT_DIM>;

  using event_handler_t = typename ddp_base_t::event_handler_t;
  using derivatives_base_t = typename ddp_base_t::derivatives_base_t;
  using constraint_base_t = typename ddp_base_t::constraint_base_t;
  using cost_function_base_t = typename ddp_base_t::cost_function_base_t;
  using operating_trajectories_base_t = typename ddp_base_t::operating_trajectories_base_t;
  using rollout_base_t = typename ddp_base_t::rollout_base_t;

  /**
   * Default constructor.
   */
  MPC_ILQR() = default;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of ILQR.
   * @param [in] partitioningTimes: This will be used as the initial time partitioning. As the MPC progresses the internal
   * partitioningTimes will be shifted in time automatically.
   * @param [in] ilqrSettings: Structure containing the settings for the ILQR algorithm.
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  MPC_ILQR(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
           const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
           const scalar_array_t& partitioningTimes, const ILQR_Settings& ilqrSettings = ILQR_Settings(),
           const MPC_Settings& mpcSettings = MPC_Settings(), const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~MPC_ILQR() = default;

  /**
   * Gets the ILQR settings structure.
   *
   * @return ILQR settings structure
   */
  virtual ILQR_Settings& ilqrSettings();

  ilqr_t* getSolverPtr() override;

  const ilqr_t* getSolverPtr() const override;

  void calculateController(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime) override;

 protected:
  /***********
   * Variables
   ***********/
  std::unique_ptr<ilqr_t> ilqrPtr_;
};

}  // namespace ocs2

#include "implementation/MPC_ILQR.h"
