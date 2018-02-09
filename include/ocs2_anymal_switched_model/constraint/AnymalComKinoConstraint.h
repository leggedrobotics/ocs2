#ifndef ANYMAL_COMKINOCONSTRAINT_H_
#define ANYMAL_COMKINOCONSTRAINT_H_

#include <c_switched_model_interface/constraint/ComKinoConstraintBase.h>
#include <c_switched_model_interface/core/Options.h>
#include <c_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

class AnymalComKinoConstraint : public switched_model::ComKinoConstraintBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::ComKinoConstraintBase<12> Base;

	AnymalComKinoConstraint(const scalar_t& gravitationalAcceleration=9.81,
			const switched_model::Options& options = switched_model::Options());

	AnymalComKinoConstraint(const AnymalComKinoConstraint& rhs);

	~AnymalComKinoConstraint() {}

	AnymalComKinoConstraint* clone() const override;

private:

};

} //end of namespace anymal

#endif /* ANYMAL_COMKINOCONSTRAINT_H_ */
