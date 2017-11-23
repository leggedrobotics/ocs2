/*
 * AnymalComKinoDynamicsDerivative.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamicsDerivative.h"

namespace anymal {

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const std::array<bool,4>& stanceLegs,
		const double& gravitationalAcceleration, const switched_model::Options& options,
		const switched_model::FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner,
		const std::vector<switched_model::EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints)

: Base(std::shared_ptr<AnymalKinematics::Base>(new AnymalKinematics),
		std::shared_ptr<AnymalCom::Base>(new AnymalCom),
		stanceLegs, gravitationalAcceleration,
		options, feetZDirectionPlanner, endEffectorStateConstraints)
{}

} //end of namespace anymal
