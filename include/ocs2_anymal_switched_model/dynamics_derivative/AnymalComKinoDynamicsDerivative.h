#ifndef ANYMAL_COMKINODYNAMICSDERIVATIVE_H_
#define ANYMAL_COMKINODYNAMICSDERIVATIVE_H_

#include <c_switched_model_interface/dynamics_derivative/ComKinoDynamicsDerivativeBase.h>
#include <c_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

class AnymalComKinoDynamicsDerivative : public switched_model::ComKinoDynamicsDerivativeBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::ComKinoDynamicsDerivativeBase<12> Base;

	AnymalComKinoDynamicsDerivative(const switched_model::Model_Settings& options = switched_model::Model_Settings());

	AnymalComKinoDynamicsDerivative(const AnymalComKinoDynamicsDerivative& rhs);

	~AnymalComKinoDynamicsDerivative() {}

	AnymalComKinoDynamicsDerivative* clone() const override;

private:

};

} //end of namespace anymal

#endif /* end of include guard: ANYMAL_COMKINODYNAMICSDERIVATIVE_H_ */
