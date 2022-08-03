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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_oc/rollout/RolloutSettings.h>

namespace ocs2 {

/**
 * @brief The RaisimRolloutSettings class contains all settings relevant for the RaisimRollout
 */
class RaisimRolloutSettings {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor that fully initializes all settings
   * @param[in] rolloutSettings The preconfigured rollout settings for a standard ocs2 rollout
   * @param[in] setSimulatorStateOnRolloutRunAlways  Whether or not to always set the starting state of the rollout to the simulator
   * @param[in] setSimulatorStateOnRolloutRunOnce Whether or not to set the starting state to the simulator at the next rollout call only
   * @param[in] controlDecimation Frequency decimation of control evaluation w.r.t. timeStep
   * @param[in] orderedJointNames A specific joint order to be used by Raisim instead of the default from URDF parsing. Parents must be
   * named before children, names must be identical to URDF joints
   * @param[in] controlMode Whether raisim should only apply generalized forces or also use an internal PD controller
   * @param[in] pGains Gains on generalized position for the Raisim-internal PD controller (if controlMode != FORCE_AND_TORQUE)
   * @param[in] dGains Gains on generalized velocity for the Raisim-internal PD controller (if controlMode != FORCE_AND_TORQUE)
   * @param[in] generateTerrain Whether or not uneven terrain should be used inside raisim
   * @param[in] terrainRoughness: z scale of the raisim heightmap
   * @param[in] terrainSeed: Seed for generating the random terrain
   * @param[in] raisimServer: Whether or not to launch the Raisim server, e.g. needed for visualization with Raisim Unity
   * @param[in] portNumber: Port number for the server, e.g. change if you have multiple instances of the RaisimRollout class
   */
  explicit RaisimRolloutSettings(rollout::Settings rolloutSettings = rollout::Settings(), bool setSimulatorStateOnRolloutRunAlways = true,
                                 bool setSimulatorStateOnRolloutRunOnce = false, int controlDecimation = 1,
                                 std::vector<std::string> orderedJointNames = {},
                                 raisim::ControlMode::Type controlMode = raisim::ControlMode::FORCE_AND_TORQUE,
                                 const Eigen::VectorXd& pGains = Eigen::VectorXd::Zero(0),  // NOLINT(modernize-pass-by-value)
                                 const Eigen::VectorXd& dGains = Eigen::VectorXd::Zero(0),  // NOLINT(modernize-pass-by-value)
                                 bool generateTerrain = false, scalar_t terrainRoughness = 1.0, int terrainSeed = 1,
                                 bool raisimServer = true, int portNumber = 8080)
      : rolloutSettings_(std::move(rolloutSettings)),
        setSimulatorStateOnRolloutRunAlways_(setSimulatorStateOnRolloutRunAlways),
        setSimulatorStateOnRolloutRunOnce_(setSimulatorStateOnRolloutRunOnce),
        controlDecimation_(controlDecimation),
        orderedJointNames_(std::move(orderedJointNames)),
        controlMode_(controlMode),
        pGains_(pGains),
        dGains_(dGains),
        generateTerrain_(generateTerrain),
        terrainRoughness_(terrainRoughness),
        terrainSeed_(terrainSeed),
        raisimServer_(raisimServer),
        portNumber_(portNumber) {}

  /**
   * @brief Constructor taking directly a settings file for initialization
   * @param fileName The name of the .info file
   * @param fieldName The name of the raisim settings inside the file
   * @param verbose Whether or not to print the loaded settings
   */
  RaisimRolloutSettings(const std::string& fileName, const std::string& fieldName, bool verbose = true) : RaisimRolloutSettings() {
    loadSettings(fileName, fieldName, verbose);
  }

  /**
   * @brief Load settings from a configuration file. Missing fields will remain at the default values.
   * @param[in] filename File name which contains the configuration data
   * @param[in] fieldName Field name which contains the configuration data
   * @param[in] verbose Flag to determine whether to print out the loaded settings or not
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true);

 public:
  rollout::Settings rolloutSettings_;

  bool setSimulatorStateOnRolloutRunAlways_;
  bool setSimulatorStateOnRolloutRunOnce_;
  int controlDecimation_;
  std::vector<std::string> orderedJointNames_;
  raisim::ControlMode::Type controlMode_;
  Eigen::VectorXd pGains_;
  Eigen::VectorXd dGains_;
  bool generateTerrain_;
  scalar_t terrainRoughness_;
  int terrainSeed_;
  bool raisimServer_;
  int portNumber_;
};

inline void RaisimRolloutSettings::loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  rolloutSettings_ = rollout::loadSettings(filename, fieldName, verbose);

  if (verbose) {
    std::cerr << "\n#### RaisimRolloutSettings:\n";
    std::cerr << "#### =============================================================================" << std::endl;
  }

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);
  const std::string raisimFieldName = fieldName + ".raisim_rollout";

  loadData::loadPtreeValue(pt, setSimulatorStateOnRolloutRunAlways_, raisimFieldName + ".setSimulatorStateOnRolloutRunAlways", verbose);
  loadData::loadPtreeValue(pt, setSimulatorStateOnRolloutRunOnce_, raisimFieldName + ".setSimulatorStateOnRolloutRunOnce", verbose);
  loadData::loadPtreeValue(pt, controlDecimation_, raisimFieldName + ".controlDecimation", verbose);
  loadData::loadStdVector(filename, raisimFieldName + ".orderedJointNames", orderedJointNames_, verbose);
  int controlModeInt = static_cast<int>(controlMode_);  // save default
  loadData::loadPtreeValue(pt, controlModeInt, raisimFieldName + ".controlMode", verbose);
  controlMode_ = static_cast<raisim::ControlMode::Type>(controlModeInt);

  std::vector<scalar_t> pGainsVec, dGainsVec;
  loadData::loadStdVector(filename, raisimFieldName + ".pGains", pGainsVec, verbose);
  if (!pGainsVec.empty()) {
    pGains_ = Eigen::Map<Eigen::VectorXd>(pGainsVec.data(), pGainsVec.size());
  }
  loadData::loadStdVector(filename, raisimFieldName + ".dGains", dGainsVec, verbose);
  if (!dGainsVec.empty()) {
    dGains_ = Eigen::Map<Eigen::VectorXd>(dGainsVec.data(), dGainsVec.size());
  }

  loadData::loadPtreeValue(pt, generateTerrain_, raisimFieldName + ".generateTerrain", verbose);
  loadData::loadPtreeValue(pt, terrainRoughness_, raisimFieldName + ".terrainRoughness", verbose);
  loadData::loadPtreeValue(pt, terrainSeed_, raisimFieldName + ".terrainSeed", verbose);
  loadData::loadPtreeValue(pt, raisimServer_, raisimFieldName + ".raisimServer", verbose);
  loadData::loadPtreeValue(pt, portNumber_, raisimFieldName + ".portNumber", verbose);
}

}  // namespace ocs2
