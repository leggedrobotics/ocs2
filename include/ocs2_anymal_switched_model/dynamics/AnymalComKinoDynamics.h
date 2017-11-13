#ifndef ANYMAL_COMKINODYNAMICS_H_
#define ANYMAL_COMKINODYNAMICS_H_

#include <c_switched_model_interface/dynamics/ComKinoDynamicsBase.h>
#include <ocs2_anymal_switched_model/AnymalSwitchedModel.h>

namespace anymal {

class AnymalComKinoDynamics : public ComKinoDynamicsBase<12>
{
public:
	typedef ComKinoDynamicsBase<12> Base;

	AnymalComKinoDynamics(const std::array<bool,4>& stanceLegs,
		const double& gravitationalAcceleration=9.81, const Base::Options& options = Options(),
		const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner=NULL,
		const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints = std::vector<EndEffectorConstraintBase::Ptr>())

	: Base(kinematics_, comDynamics_, stanceLegs, gravitationalAcceleration,
			options, feetZDirectionPlanner, endEffectorStateConstraints)
	{}

protected:
	AnymalKinematics kinematics_;
	AnymalCom comDynamics_;

};

} //end of namespace anymal

#endif /* end of include guard: ANYMAL_COMKINODYNAMICS_H_ */
