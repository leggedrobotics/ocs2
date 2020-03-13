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

// OCS2
#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

namespace ocs2 {

/**
 * This class implements an interface class to all the robotic examples.
 *
 * The lifetime of the returned objects is tied to the lifetime of the robot interface.
 * The exposed objects are not thread-safe and should be cloned to get an exclusive copy.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RobotInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dynamics_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using dynamics_derivatives_t = DerivativesBase<STATE_DIM, INPUT_DIM>;
  using cost_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using operating_point_t = SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM>;

  /**
   * Destructor
   */
  virtual ~RobotInterface() = default;

  /**
   * @brief getDynamics
   * @return a reference to the interal system dynamics
   */
  virtual const dynamics_t& getDynamics() const = 0;

  /**
   * @brief getDynamicsDerivatives
   * @return a reference to the internal system dynamics derivatives
   */
  virtual const dynamics_derivatives_t& getDynamicsDerivatives() const = 0;

  /**
   * @brief getCost
   * @return reference to internal cost function
   */
  virtual const cost_t& getCost() const = 0;

  /**
   * @brief getConstraintPtr
   * @return pointer to internal constraint object. Can be nullptr in case of zero constraints
   */
  virtual const constraint_t* getConstraintPtr() const { return nullptr; }

  /**
   * @brief getOperatingPoints
   * @return reference to the internal operating point
   */
  virtual const operating_point_t& getOperatingPoints() const = 0;
};

}  // namespace ocs2
