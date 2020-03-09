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

#include <Eigen/Core>
#include <raisim/World.hpp>

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_core/Dimensions.h>

namespace ocs2 {
namespace ballbot {

/**
 * @brief Conversions between raiSim and ocs2 for the ballbot system
 *
 * Ballbot state: [ball x, ball y, base yaw, base pitch, base roll; <time derivatives of all positions>]
 * Ballbot input: [3-d ball torque]
 * raisim q: [3-d base position, base quaternion (w x y z), quaternion of spherical joint (w x y z)]
 * raisim dq: [3-d base linear vel in world, 3-d base angular vel in world, 3-d spherical joint velocities]
 * raisim generalizedForce [ 6-d wrench on base, 3-d torque through spherical joint]
 */
class BallbotRaisimConversions final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dim_t = Dimensions<STATE_DIM_, INPUT_DIM_>;
  using state_vector_t = dim_t::state_vector_t;
  using input_vector_t = dim_t::input_vector_t;

  //! Constructor
  BallbotRaisimConversions();

  //! Default destructor
  ~BallbotRaisimConversions() = default;

  /**
   * @brief Calculate the position of the ball from generalized coordinates
   * @param q Raisim generalized positions
   * @return The 3d position vector of the ball's center in the world frame
   */
  Eigen::Vector3d ballCenterInWorld(const Eigen::VectorXd& q) const;

  /**
   * @brief Convert ocs2 state into raisim generalized coordinates and velocities
   * @param[in] state ocs2 ballbot state
   * @param[in] input ocs2 ballbot input (unused)
   * @return The pair {q, dq} consisting of raisim generlized coordinates and velocities
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const state_vector_t& state, const input_vector_t& input) const;

  /**
   * @brief Convert raisim state to ocs2 state
   * @param[in] q Raisim generalized coordinate
   * @param[in] dq Raisim generalized velocity
   * @return ocs2 state vector
   */
  state_vector_t raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  /**
   * @brief Convert ocs2 input to raisim generalized force
   * @param[in] time Current time
   * @param[in] input ocs2 control input
   * @param[in] state ocs2 state
   * @param[in] q Raisim generalized coordinate
   * @param[in] dq Raisim generalized velocities
   * @return Vector of generalized forces for raisim
   */
  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const input_vector_t& input, const state_vector_t& state,
                                                const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

 private:
  double ballRadius_;
  double omniWheelRadius_;
  double distanceBaseToBallCenter_;
  Eigen::Vector3d rbaseBallInBase_;
};

}  // namespace ballbot
}  // namespace ocs2
