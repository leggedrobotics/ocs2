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

#include <ros/package.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>

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
ocs2::PinocchioInterface MobileManipulatorInterface::buildPinocchioInterface(const std::string& urdfPath) {
  // add 3 DOF for wheelbase
  pinocchio::JointModelComposite rootJoint(3);
  rootJoint.addJoint(pinocchio::JointModelPX());
  rootJoint.addJoint(pinocchio::JointModelPY());
  rootJoint.addJoint(pinocchio::JointModelRZ());

  return ocs2::getPinocchioInterfaceFromUrdfFile(urdfPath, rootJoint);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::loadSettings(const std::string& taskFile) {
  std::cerr << "Load Pinocchio model from " << urdfPath_ << '\n';

  pinocchioInterfacePtr_.reset(new ocs2::PinocchioInterface(buildPinocchioInterface(urdfPath_)));
  std::cerr << *pinocchioInterfacePtr_;

  /*
   * DDP-MPC settings
   */
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");

  bool recompileLibraries;
  ocs2::loadData::loadCppDataType(taskFile_, "model_settings.recompileLibraries", recompileLibraries);

  /*
   * Dynamics
   */
  dynamicsPtr_.reset(new MobileManipulatorDynamics(ocs2::castToCppAd(*pinocchioInterfacePtr_)));
  dynamicsPtr_->initialize("mobile_manipulator_dynamics", libraryFolder_, recompileLibraries, true);

  /*
   * Rollout
   */
  auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  costPtr_.reset(new MobileManipulatorCost(*pinocchioInterfacePtr_, taskFile, libraryFolder_, recompileLibraries));

  /*
   * Constraints
   */
  constraintPtr_.reset(new ocs2::ConstraintBase());

  /*
   * Initialization
   */
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;
  operatingPointPtr_.reset(new ocs2::OperatingPoints(initialState_, ocs2::vector_t::Zero(INPUT_DIM)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::MPC_DDP> MobileManipulatorInterface::getMpc() {
  return std::unique_ptr<ocs2::MPC_DDP>(new ocs2::MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                                                          operatingPointPtr_.get(), ddpSettings_, mpcSettings_));
}

}  // namespace mobile_manipulator
