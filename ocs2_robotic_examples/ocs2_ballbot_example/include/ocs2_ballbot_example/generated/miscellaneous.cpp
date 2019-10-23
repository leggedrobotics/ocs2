#include "miscellaneous.h"
#include <iit/rbd/utils.h>

using namespace iit::Ballbot;
using namespace iit::Ballbot::dyn;

iit::rbd::Vector3d iit::Ballbot::getWholeBodyCOM(const InertiaProperties& inertiaProps, const HomogeneousTransforms& ht) {
  iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

  HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
  tmpX = tmpX * ht.fr_world_X_fr_dummy_ball1;
  tmpSum += inertiaProps.getMass_dummy_ball1() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_dummy_ball1()));

  tmpX = tmpX * ht.fr_dummy_ball1_X_fr_ball;
  tmpSum += inertiaProps.getMass_ball() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_ball()));

  tmpX = tmpX * ht.fr_ball_X_fr_dummy_base1;
  tmpSum += inertiaProps.getMass_dummy_base1() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_dummy_base1()));

  tmpX = tmpX * ht.fr_dummy_base1_X_fr_dummy_base2;
  tmpSum += inertiaProps.getMass_dummy_base2() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_dummy_base2()));

  tmpX = tmpX * ht.fr_dummy_base2_X_fr_base;
  tmpSum += inertiaProps.getMass_base() * (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_base()));

  return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::Ballbot::getWholeBodyCOM(const InertiaProperties& inertiaProps, const JointState& q, HomogeneousTransforms& ht) {
  // First updates the coordinate transforms that will be used by the routine
  ht.fr_world_X_fr_dummy_ball1(q);
  ht.fr_dummy_ball1_X_fr_ball(q);
  ht.fr_ball_X_fr_dummy_base1(q);
  ht.fr_dummy_base1_X_fr_dummy_base2(q);
  ht.fr_dummy_base2_X_fr_base(q);

  // The actual calculus
  return getWholeBodyCOM(inertiaProps, ht);
}
