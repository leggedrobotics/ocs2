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

#include <ocs2_ballbot/generated/forward_dynamics.h>
#include <ocs2_ballbot/generated/inertia_properties.h>
#include <ocs2_ballbot/generated/inverse_dynamics.h>
#include <ocs2_ballbot/generated/jsim.h>
#include <ocs2_ballbot/generated/transforms.h>
#include <ocs2_robotic_tools/rbd_libraries/robcogen/iit/rbd/rbd.h>
#include <ocs2_robotic_tools/rbd_libraries/robcogen/iit/rbd/traits/TraitSelector.h>

TEST(RobcogenModelTest, RobcogenModelTest) {
  // initialize random joint state
  iit::Ballbot::tpl::JointState<double> q, qd, qdd, tau;
  q << 0.5, 0.7, 3.0, 2.0, -1.0;
  qd << 0.5, 0.7, 3.0, 2.0, -1.0;
  qdd.setZero();
  tau << 3.5, 4.5, 6.5;

  Eigen::Matrix<double, 5, 1> G_terms, C_terms;
  G_terms.setZero();
  C_terms.setZero();

  // define inertia properties, force transforms and Motion transform
  using trait_t = typename iit::rbd::tpl::TraitSelector<double>::Trait;
  iit::Ballbot::dyn::tpl::InertiaProperties<trait_t> inertias;
  iit::Ballbot::tpl::ForceTransforms<trait_t> forceTransforms;
  iit::Ballbot::tpl::MotionTransforms<trait_t> transforms;

  // compute gravity terms, coriolis and centrifugal terms
  iit::Ballbot::dyn::tpl::InverseDynamics<trait_t> invdyn(inertias, transforms);
  invdyn.G_terms(G_terms, q);
  invdyn.C_terms(C_terms, q, qd);

  // compute result of the forward dynamics method
  iit::Ballbot::dyn::tpl::ForwardDynamics<trait_t> forward_dyn(inertias, transforms);
  forward_dyn.fd(qdd, q, qd, tau);

  // print results
  std::cout << "accelerations from forward dynamics method: " << qdd.transpose() << std::endl;
  std::cout << "gravity terms: " << G_terms.transpose() << std::endl;

  q << 0.5, 0.7, 3.0, 2.0, -1.0;
  qd << 0.5, 0.7, 12.0, 2.0, -1.0;
  qdd.setZero();
  tau << 10.5, 4.5, 6.5;
  forward_dyn.fd(qdd, q, qd, tau);

  std::cout << "accelerations from forward dynamics method (second test): " << qdd.transpose() << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
