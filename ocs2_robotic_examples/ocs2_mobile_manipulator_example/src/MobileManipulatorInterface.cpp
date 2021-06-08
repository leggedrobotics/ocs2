/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ros/package.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  urdfPath_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface MobileManipulatorInterface::buildPinocchioInterface(const std::string& urdfPath) {
  // add 3 DOF for wheelbase
  pinocchio::JointModelComposite rootJoint(3);
  rootJoint.addJoint(pinocchio::JointModelPX());
  rootJoint.addJoint(pinocchio::JointModelPY());
  rootJoint.addJoint(pinocchio::JointModelRZ());

  return getPinocchioInterfaceFromUrdfFile(urdfPath, rootJoint);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::loadSettings(const std::string& taskFile) {
  std::cerr << "Load Pinocchio model from " << urdfPath_ << '\n';

  pinocchioInterfacePtr_.reset(new PinocchioInterface(buildPinocchioInterface(urdfPath_)));
  std::cerr << *pinocchioInterfacePtr_;

  bool useCaching = true;
  bool recompileLibraries = true;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### model_settings: \n";
  std::cerr << "#### =============================================================================\n";
  loadData::loadPtreeValue(pt, useCaching, "model_settings.useCaching", true);
  loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  std::cerr << " #### =============================================================================" << std::endl;

  /*
   * DDP-MPC settings
   */
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

  /*
   * Dynamics
   */
  dynamicsPtr_.reset(new MobileManipulatorDynamics("mobile_manipulator_dynamics", libraryFolder_, recompileLibraries, true));

  /*
   * Rollout
   */
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  costPtr_.reset(new MobileManipulatorCost(*pinocchioInterfacePtr_, taskFile, useCaching, libraryFolder_, recompileLibraries));

  /*
   * Constraints
   */
  constraintPtr_.reset(new ConstraintBase());

  /*
   * Initialization state
   */
  initializerPtr_.reset(new DefaultInitializer(INPUT_DIM));

  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_DDP> MobileManipulatorInterface::getMpc() {
  return std::unique_ptr<MPC_DDP>(new MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                                              initializerPtr_.get(), ddpSettings_, mpcSettings_));
}

}  // namespace mobile_manipulator
}  // namespace ocs2
