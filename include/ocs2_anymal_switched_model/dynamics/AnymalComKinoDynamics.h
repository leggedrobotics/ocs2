#ifndef ANYMAL_COMKINODYNAMICS_H_
#define ANYMAL_COMKINODYNAMICS_H_

#include <c_switched_model_interface/dynamics/ComKinoDynamicsBase.h>
#include <c_switched_model_interface/core/Options.h>
#include <c_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

class AnymalComKinoDynamics : public switched_model::ComKinoDynamicsBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::ComKinoDynamicsBase<12> Base;

	AnymalComKinoDynamics(const scalar_t& gravitationalAcceleration=9.81,
			const switched_model::Options& options = switched_model::Options());

	AnymalComKinoDynamics(const AnymalComKinoDynamics& rhs);

	~AnymalComKinoDynamics() {}

	AnymalComKinoDynamics* clone() const  override;

private:


};

} //end of namespace anymal

#endif /* end of include guard: ANYMAL_COMKINODYNAMICS_H_ */
