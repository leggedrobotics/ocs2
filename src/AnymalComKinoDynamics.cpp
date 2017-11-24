/*
 * AnymalComKinoDynamics.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h"

namespace anymal {

AnymalComKinoDynamics::AnymalComKinoDynamics(const std::array<bool,4>& stanceLegs,
		const double& gravitationalAcceleration, const switched_model::Options& options,
		const switched_model::FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner,
		const std::vector<switched_model::EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints)

: Base(std::shared_ptr<AnymalKinematics::Base>(new AnymalKinematics),
		std::shared_ptr<AnymalCom::Base>(new AnymalCom),
		stanceLegs, gravitationalAcceleration, options,
		feetZDirectionPlanner, endEffectorStateConstraints)
{}

std::shared_ptr<AnymalComKinoDynamics::Base::Base> AnymalComKinoDynamics::clone() const {

	return std::allocate_shared< AnymalComKinoDynamics, Eigen::aligned_allocator<AnymalComKinoDynamics> > (
			Eigen::aligned_allocator<AnymalComKinoDynamics>(), *this);
}

} //end of namespace anymal
