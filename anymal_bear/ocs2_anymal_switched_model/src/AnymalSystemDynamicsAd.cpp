//
// Created by rgrandia on 21.10.19.
//

#include "ocs2_anymal_switched_model/dynamics/AnymalSystemDynamicsAd.h"

#include "ocs2_anymal_switched_model/kinematics/AnymalKinematics.h"
#include "ocs2_anymal_switched_model/dynamics/AnymalCom.h"

namespace anymal {

AnymalSystemDynamicsAd::AnymalSystemDynamicsAd(bool recompileModel)

    : Base(AnymalKinematicsAd(), AnymalComAd(), recompileModel)
{}

AnymalSystemDynamicsAd::AnymalSystemDynamicsAd(const AnymalSystemDynamicsAd& rhs)
    : Base(rhs)
{}

AnymalSystemDynamicsAd* AnymalSystemDynamicsAd::clone() const {
  return new AnymalSystemDynamicsAd(*this);
}

} // end of namespace anymal