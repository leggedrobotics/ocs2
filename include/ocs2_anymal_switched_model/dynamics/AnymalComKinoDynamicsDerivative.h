#ifndef ANYMAL_COMKINODYNAMICSDERIVATIVE_H_
#define ANYMAL_COMKINODYNAMICSDERIVATIVE_H_

#include <c_switched_model_interface/dynamics_derivative/ComKinoDynamicsDerivativeBase.h>
#include <ocs2_anymal_switched_model/AnymalSwitchedModel.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

class AnymalComKinoDynamicsDerivative : public switched_model::ComKinoDynamicsDerivativeBase<12>
{
private:
	typedef ComKinoDynamicsDerivativeBase<12> Base;

public:
	AnymalComKinoDynamicsDerivative(const std::array<bool,4>& stanceLegs,
		const double& gravitationalAcceleration=9.81, const switched_model::Options& options = Options(),
		const switched_model::FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner=NULL,
		const std::vector<switched_model::EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints = std::vector<switched_model::EndEffectorConstraintBase::Ptr>())

	: Base(kinematics_.clone(), comDynamics_.clone(), stanceLegs, gravitationalAcceleration,
			options, feetZDirectionPlanner, endEffectorStateConstraints)
	{}

protected:
	AnymalKinematics kinematics_;
	AnymalCom comDynamics_;

};

} //end of namespace anymal

#endif /* end of include guard: ANYMAL_COMKINODYNAMICSDERIVATIVE_H_ */
