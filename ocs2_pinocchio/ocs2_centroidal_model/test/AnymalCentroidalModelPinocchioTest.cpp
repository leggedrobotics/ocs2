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

#include <gtest/gtest.h>

#include <ocs2_centroidal_model/example/anymal/definitions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioInterface.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_core/misc/Benchmark.h>

#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

using namespace ocs2;

using CentroidalModelType = CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType;

static CentroidalModelPinocchioInterface<scalar_t> getAnymalCentroidalModelInterface1() {
  // build a joint model having just 2 joints: one translational joint and one spherical joint
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  PinocchioInterface pinocchioInterface(getPinocchioInterfaceFromUrdfFile(anymalUrdfPath, jointComposite));
  const size_t nq = pinocchioInterface.getModel().nq;
  CentroidalModelPinocchioInterface<scalar_t> anymalCentroidalModelInterface(
          CentroidalModelType::FullCentroidalDynamics,
          vector_t::Zero(nq), anymal3DofContactNames,
          anymal6DofContactNames, pinocchioInterface);
  return anymalCentroidalModelInterface;
}

static CentroidalModelPinocchioInterface<scalar_t> getAnymalCentroidalModelInterface2() {
  // build a joint model having just 2 joints: one translational joint and one spherical joint
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  PinocchioInterface pinocchioInterface(getPinocchioInterfaceFromUrdfFile(anymalUrdfPath, jointComposite));
  const size_t nq = pinocchioInterface.getModel().nq;
  CentroidalModelPinocchioInterface<scalar_t> anymalCentroidalModelInterface(
          CentroidalModelType::SingleRigidBodyDynamics,
          anymalInitialState.tail(nq), anymal3DofContactNames,
          anymal6DofContactNames, pinocchioInterface);
  return anymalCentroidalModelInterface;
}

TEST(AnymalCentroidalModelPinocchioInterfaceTest, CentroidalModelPinocchioTest) {
  // Full Centroidal Dynamics Model (FCD)
  auto anymalFCD = getAnymalCentroidalModelInterface1();
  auto &modelFCD = anymalFCD.getRobotModel();
  auto &dataFCD = anymalFCD.getRobotData();
  auto &centroidalModelInfoFCD = anymalFCD.getCentroidalModelInfo();

  // Single Rigid Body Dynamics Model (SRBD)
  auto anymalSRBD = getAnymalCentroidalModelInterface2();
  auto &modelSRBD = anymalSRBD.getRobotModel();
  auto &dataSRBD = anymalSRBD.getRobotData();
  auto &centroidalModelInfoSRBD = anymalSRBD.getCentroidalModelInfo();

  const size_t nq = modelFCD.nq;
  const size_t nv = modelFCD.nv;
  std::cerr << "Pinocchio Joint Positions Size: " << nq << std::endl;
  std::cerr << "Pinocchio Joint Velocities Size: " << nv << std::endl;

  // check if the frames of the feet exist
  bool exist_lf_foot = modelFCD.existFrame("LF_FOOT");
  bool exist_lh_foot = modelFCD.existFrame("LH_FOOT");
  bool exist_rf_foot = modelFCD.existFrame("RF_FOOT");
  bool exist_rh_foot = modelFCD.existFrame("RH_FOOT");
  bool exist_base = modelFCD.existFrame("base");
  std::cerr << "exist_lf_foot: " << exist_lf_foot << std::endl;
  std::cerr << "exist_lh_foot: " << exist_lh_foot << std::endl;
  std::cerr << "exist_rf_foot: " << exist_rf_foot << std::endl;
  std::cerr << "exist_rh_foot: " << exist_rh_foot << std::endl;
  std::cerr << "exist_base: " << exist_base << std::endl;

  // Get indices of frames
  int frame_lf_foot = modelFCD.getFrameId("LF_FOOT");
  int frame_lh_foot = modelFCD.getFrameId("LH_FOOT");
  int frame_rf_foot = modelFCD.getFrameId("RF_FOOT");
  int frame_rh_foot = modelFCD.getFrameId("RH_FOOT");
  int joint_LF_KFE = modelFCD.getJointId("LF_KFE");
  int frame_BASE = modelFCD.getFrameId("base");

  // Compare FCD model with SRBD model
  vector_t nominalState = anymalInitialState;
  nominalState.head(6) = vector_t::Random(6);
  vector_t randomInput = vector_t::Random(anymal::INPUT_DIM);
  vector_t zeroInput = vector_t::Zero(anymal::INPUT_DIM);
  benchmark::RepeatedTimer timer_;

  //-- Comparison based on nominal pinocchio joint positions and zero velocities --//
  timer_.startTimer();
  anymalFCD.updatePinocchioJointPositions(nominalState);
  anymalFCD.updatePinocchioJointVelocities(nominalState, zeroInput);
  timer_.endTimer();
  double durationFCD = timer_.getTotalInMilliseconds();

  timer_.reset();
  timer_.startTimer();
  anymalSRBD.updatePinocchioJointPositions(nominalState);
  anymalSRBD.updatePinocchioJointVelocities(nominalState, zeroInput);
  timer_.endTimer();
  double durationSRBD = timer_.getTotalInMilliseconds();

  std::cerr << "qPinocchio: " << centroidalModelInfoFCD.qPinocchio.transpose() << '\n';
  std::cerr << "vPinocchio: " << centroidalModelInfoFCD.vPinocchio.transpose() << '\n';
  std::cerr << "Duration FCD model: " << durationFCD << '\n';
  std::cerr << "Duration SRBD model: " << durationSRBD << '\n';
  EXPECT_TRUE(centroidalModelInfoFCD.qPinocchio.isApprox(centroidalModelInfoSRBD.qPinocchio));
  EXPECT_TRUE(centroidalModelInfoFCD.vPinocchio.isApprox(centroidalModelInfoSRBD.vPinocchio));
  ASSERT_GT(durationFCD, durationSRBD);

  //-- Comparison based on a perturbed base pose and zero velocities --//
  auto perturbedState = nominalState;
  perturbedState.segment<6>(6) = vector_t::Random(6);  // different base pose

  timer_.reset();
  timer_.startTimer();
  anymalFCD.updatePinocchioJointPositions(perturbedState);
  anymalFCD.updatePinocchioJointVelocities(perturbedState, zeroInput);
  timer_.endTimer();
  durationFCD = timer_.getTotalInMilliseconds();

  timer_.reset();
  timer_.startTimer();
  anymalSRBD.updatePinocchioJointPositions(perturbedState);
  anymalSRBD.updatePinocchioJointVelocities(perturbedState, zeroInput);
  timer_.endTimer();
  durationSRBD = timer_.getTotalInMilliseconds();

  std::cerr << "qPinocchio: " << centroidalModelInfoFCD.qPinocchio.transpose() << '\n';
  std::cerr << "vPinocchio: " << centroidalModelInfoFCD.vPinocchio.transpose() << '\n';
  std::cerr << "Ab_FCD: \n" << centroidalModelInfoFCD.Ab << '\n';
  std::cerr << "Ab_SRBD: \n" << centroidalModelInfoSRBD.Ab << '\n';
  std::cerr << "Duration FCD model: " << durationFCD << '\n';
  std::cerr << "Duration SRBD model: " << durationSRBD << '\n';
  EXPECT_TRUE(centroidalModelInfoFCD.qPinocchio.isApprox(centroidalModelInfoSRBD.qPinocchio));
  EXPECT_TRUE(centroidalModelInfoFCD.vPinocchio.isApprox(centroidalModelInfoSRBD.vPinocchio));
  ASSERT_GT(durationFCD, durationSRBD);

  //-- Comparison based on perturbed pinocchio joint positions and zero velocities --//
  perturbedState = nominalState + vector_t::Random(anymal::STATE_DIM);

  timer_.reset();
  timer_.startTimer();
  anymalFCD.updatePinocchioJointPositions(perturbedState);
  anymalFCD.updatePinocchioJointVelocities(perturbedState, zeroInput);
  timer_.endTimer();
  durationFCD = timer_.getTotalInMilliseconds();

  timer_.reset();
  timer_.startTimer();
  anymalSRBD.updatePinocchioJointPositions(perturbedState);
  anymalSRBD.updatePinocchioJointVelocities(perturbedState, zeroInput);
  timer_.endTimer();
  durationSRBD = timer_.getTotalInMilliseconds();

  std::cerr << "qPinocchio: " << centroidalModelInfoFCD.qPinocchio.transpose() << '\n';
  std::cerr << "vPinocchio: " << centroidalModelInfoFCD.vPinocchio.transpose() << '\n';
  std::cerr << "Duration FCD model: " << durationFCD << '\n';
  std::cerr << "Duration SRBD model: " << durationSRBD << '\n';
  EXPECT_TRUE(centroidalModelInfoFCD.qPinocchio.isApprox(centroidalModelInfoSRBD.qPinocchio));
  EXPECT_FALSE(centroidalModelInfoFCD.vPinocchio.isApprox(centroidalModelInfoSRBD.vPinocchio));
  ASSERT_GT(durationFCD, durationSRBD);

  //-- Comparison based on nominal pinocchio joint positions and random velocities --//
  timer_.reset();
  timer_.startTimer();
  anymalFCD.updatePinocchioJointPositions(nominalState);
  anymalFCD.updatePinocchioJointVelocities(nominalState, randomInput);
  timer_.endTimer();
  durationFCD = timer_.getTotalInMilliseconds();
  pinocchio::updateFramePlacements(modelFCD, dataFCD);

  timer_.reset();
  timer_.startTimer();
  anymalSRBD.updatePinocchioJointPositions(nominalState);
  anymalSRBD.updatePinocchioJointVelocities(nominalState, randomInput);
  timer_.endTimer();
  durationSRBD = timer_.getTotalInMilliseconds();
  pinocchio::updateFramePlacements(modelSRBD, dataSRBD);

  std::cerr << "qPinocchio: " << centroidalModelInfoFCD.qPinocchio.transpose() << '\n';
  std::cerr << "vPinocchio: " << centroidalModelInfoFCD.vPinocchio.transpose() << '\n';
  std::cerr << "Duration FCD model: " << durationFCD << '\n';
  std::cerr << "Duration SRBD model: " << durationSRBD << '\n';
  EXPECT_TRUE(centroidalModelInfoFCD.qPinocchio.isApprox(centroidalModelInfoSRBD.qPinocchio));
  EXPECT_FALSE(centroidalModelInfoFCD.vPinocchio.isApprox(centroidalModelInfoSRBD.vPinocchio));
  ASSERT_GT(durationFCD, durationSRBD);

  // Compute absolute positions of joints in world frame
  for (int k = 0; k < modelFCD.njoints; ++k) {
    std::cerr << modelFCD.names[k] << "\t translation: " << dataFCD.oMi[k].translation().transpose() << std::endl;
  }

  // Compute absolute positions of frames in world frame
  Eigen::Matrix<double, 3, 1> pos_lf_frame = anymalFCD.positionWorldToContactPointInWorldFrame(0);
  Eigen::Matrix<double, 3, 1> pos_rf_frame = anymalFCD.positionWorldToContactPointInWorldFrame(1);
  Eigen::Matrix<double, 3, 1> pos_lh_frame = anymalFCD.positionWorldToContactPointInWorldFrame(2);
  Eigen::Matrix<double, 3, 1> pos_rh_frame = anymalFCD.positionWorldToContactPointInWorldFrame(3);
  Eigen::Matrix<double, 3, 1> pos_base = dataFCD.oMf[frame_BASE].translation();
  std::cerr << "pos_base " << pos_base.transpose() << std::endl;
  std::cerr << "pos_lf_foot " << pos_lf_frame.transpose() << std::endl;
  std::cerr << "pos_lh_foot " << pos_lh_frame.transpose() << std::endl;
  std::cerr << "pos_rf_foot " << pos_rf_frame.transpose() << std::endl;
  std::cerr << "pos_rh_foot " << pos_rh_frame.transpose() << std::endl;
  EXPECT_TRUE(pos_base.isApprox(centroidalModelInfoFCD.qPinocchio.head(3)));

  // Compute absolute velocities of frames in world frame
  Eigen::Matrix<double, 3, 1> vel_lf_frame = anymalFCD.linearVelocityWorldToContactPointInWorldFrame(0);
  Eigen::Matrix<double, 3, 1> vel_rf_frame = anymalFCD.linearVelocityWorldToContactPointInWorldFrame(1);
  Eigen::Matrix<double, 3, 1> vel_lh_frame = anymalFCD.linearVelocityWorldToContactPointInWorldFrame(2);
  Eigen::Matrix<double, 3, 1> vel_rh_frame = anymalFCD.linearVelocityWorldToContactPointInWorldFrame(3);
  std::cerr << "vel_lf_foot " << vel_lf_frame.transpose() << std::endl;
  std::cerr << "vel_lh_foot " << vel_lh_frame.transpose() << std::endl;
  std::cerr << "vel_rf_foot " << vel_rf_frame.transpose() << std::endl;
  std::cerr << "vel_rh_foot " << vel_rh_frame.transpose() << std::endl;
}