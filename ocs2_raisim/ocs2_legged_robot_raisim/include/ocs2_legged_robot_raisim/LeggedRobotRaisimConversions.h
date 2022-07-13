/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <raisim/object/terrain/HeightMap.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

namespace ocs2 {
namespace legged_robot {

/**
 * Conversions between RaiSim variables and OCS2 variables for the legged robot.
 */
class LeggedRobotRaisimConversions final {
 public:
  /**
   * Constructor.
   * @param [in] pinocchioInterface : The predefined pinocchio interface for the robot.
   * @param [in] centroidalModelInfo : The centroidal model information.
   * @param [in] initialState : The initial switched model state.
   * @param [in] check : Whether to check if the variables coming from or going to RaiSim respect the actuator limits (by default false).
   */
  LeggedRobotRaisimConversions(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& centroidalModelInfo,
                               const vector_t& initialState, bool check = false)
      : check_(check),
        continuousOrientation_(initialState.segment<3>(9)),
        centroidalModelRbdConversions_(std::move(pinocchioInterface), centroidalModelInfo) {}

  /**
   * @brief Convert OCS2 switched model state to generalized coordinate and generalized velocity used by RaiSim.
   * @param [in] state : The switched model state to be converted.
   * @param [in] input : The switched model input (includes state information due to the kinematic leg model).
   * @return The corresponding {q, dq} pair that represents the simulator state.
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const vector_t& state, const vector_t& input);

  /**
   * @brief Convert RaiSim generalized coordinates and velocities to OCS2 switched model state.
   * @param [in] q : The generalized coordinate.
   * @param [in] dq : The generalized velocity.
   * @return The corresponding switched model state.
   */
  vector_t raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert OCS2 switched model input to RaiSim generalized force.
   * @param [in] time : The current time.
   * @param [in] input : The switched model input to be converted.
   * @param [in] state : The switched model state (includes reference information for PD control on acceleration level).
   * @param [in] q : The RaiSim generalized coordinate.
   * @param [in] dq : The RaiSim generalized velocity.
   * @return The corresponding generalized forces to be applied to the system.
   */
  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& dq);

  /**
   * @brief Convert OCS2 switched model input to RaiSim PD setpoints.
   * @param [in] time : The current time.
   * @param [in] input : The switched model input to be converted.
   * @param [in] state : The switched model state (includes reference information for the PD control on torque level).
   * @param [in] q : The RaiSim generalized coordinate.
   * @param [in] dq : The RaiSim generalized velocity.
   * @return The generalized position and velocities to be used as PD control setpoints by RaiSim.
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> inputToRaisimPdTargets(double time, const vector_t& input, const vector_t& state,
                                                                     const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert RaiSim joint variables to OCS2 joint variables.
   * @note This can be removed if changing the RaiSim joint order works eventually.
   * @param [in] raisimJoint : The joint variables in RaiSim order.
   * @return The joint variables in OCS2 order.
   */
  vector_t raisimJointOrderToOcs2JointOrder(const Eigen::VectorXd& raisimJoint);

  /**
   * @brief Convert OCS2 joint variables to RaiSim joint variables.
   * @note This can be removed if changing the RaiSim joint order works eventually.
   * @param [in] ocs2Joint : The joint variables in OCS2 order.
   * @return The joint variables in RaiSim order.
   */
  Eigen::VectorXd ocs2JointOrderToRaisimJointOrder(const vector_t& ocs2Joint);

  /**
   * @brief Set the terrain.
   * @param [in] terrain : The terrain represented by a RaiSim height map.
   */
  void setTerrain(const raisim::HeightMap& terrain) { terrainPtr_ = &terrain; }

  /**
   * @brief Load settings from a configuration file.
   * @param [in] fileName : File name which contains the configuration data.
   * @param [in] fieldName : Field name which contains the configuration data.
   * @param [in] verbose : Flag to determine whether to print out the loaded settings or not.
   */
  void loadSettings(const std::string& fileName, const std::string& fieldName, bool verbose = true) {
    centroidalModelRbdConversions_.loadSettings(fileName, fieldName, verbose);
  }

 protected:
  /**
   * @brief Convert OCS2 RBD state to generalized coordinate and generalized velocity used by RaiSim.
   * @param [in] rbdState : The RBD state to be converted.
   * @return The corresponding {q, dq} pair that represents the simulator state.
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> rbdStateToRaisimGenCoordGenVel(const vector_t& rbdState);

  /**
   * @brief Convert RaiSim generalized coordinates and velocities to OCS2 RBD state.
   * @param [in] q : The generalized coordinate.
   * @param [in] dq : The generalized velocity.
   * @return The corresponding RBD state.
   */
  vector_t raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert OCS2 RBD torque to RaiSim generalized force.
   * @param [in] rbdTorque : The RBD torque to be converted.
   * @return The corresponding generalized forces to be applied to the system.
   */
  Eigen::VectorXd rbdTorqueToRaisimGeneralizedForce(const vector_t& rbdTorque);

 private:
  const bool check_;
  Eigen::Vector3d continuousOrientation_;
  CentroidalModelRbdConversions centroidalModelRbdConversions_;
  const raisim::HeightMap* terrainPtr_ = nullptr;
};

}  // namespace legged_robot
}  // namespace ocs2
