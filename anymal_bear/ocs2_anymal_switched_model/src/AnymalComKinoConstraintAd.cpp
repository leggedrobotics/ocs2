#include "ocs2_anymal_switched_model/constraint/AnymalComKinoConstraintAd.h"

#include "ocs2_anymal_switched_model/core/AnymalKinematics.h"
#include "ocs2_anymal_switched_model/core/AnymalCom.h"

namespace anymal {

AnymalComKinoConstraintAd::AnymalComKinoConstraintAd(std::shared_ptr<const logic_rules_t> logicRulesPtr, const switched_model::Model_Settings& options)

: Base(AnymalKinematicsAd(), AnymalComAd(),  std::move(logicRulesPtr), options)
{}

AnymalComKinoConstraintAd::AnymalComKinoConstraintAd(const AnymalComKinoConstraintAd& rhs)
: Base(rhs)
{}

AnymalComKinoConstraintAd* AnymalComKinoConstraintAd::clone() const {
	return new AnymalComKinoConstraintAd(*this);
}

} // end of namespace anymal


