#include <gtest/gtest.h>
#include <ocs2_ballbot_example/generated/forward_dynamics.h>
#include <ocs2_ballbot_example/generated/inertia_properties.h>
#include <ocs2_ballbot_example/generated/inverse_dynamics.h>
#include <ocs2_ballbot_example/generated/jsim.h>
#include <ocs2_ballbot_example/generated/transforms.h>
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