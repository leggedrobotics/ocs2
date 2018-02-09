/*
 * SwitchedModelLogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef SWITCHEDMODELLOGICRULESBASE_H_
#define SWITCHEDMODELLOGICRULESBASE_H_

#include <ocs2_core/logic/LogicRulesBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/foot_planner/cpg/SplineCPG.h"
#include "c_switched_model_interface/foot_planner/FeetZDirectionPlanner.h"
#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

/**
 * Logic rules base class
 */
template <size_t JOINT_COORD_SIZE>
class SwitchedModelLogicRulesBase : public ocs2::LogicRulesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::LogicRulesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE> Base;

	typedef typename Base::size_array_t size_array_t;
	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::scalar_array_t scalar_array_t;
	typedef typename Base::controller_t controller_t;
	typedef typename Base::controller_array_t controller_array_t;

	typedef std::array<bool,4> contact_flags_t;

	typedef SplineCPG<scalar_t> cpg_t;
	typedef FeetZDirectionPlanner<cpg_t,scalar_t> feet_z_planner_t;
	typedef typename feet_z_planner_t::feet_cpg_ptr_t		feet_cpg_ptr_t;
	typedef typename feet_z_planner_t::feet_cpg_const_ptr_t	feet_cpg_const_ptr_t;

public:
	/**
	 * Default constructor
	 */
	SwitchedModelLogicRulesBase() = delete;

	/**
	 * Constructor
	 *
	 * @param [in] swingLegLiftOff: Maximum swing leg lift-off
	 * @param [in] swingTimeScale: The scaling factor for adapting swing leg's lift-off
	 */
	SwitchedModelLogicRulesBase(const scalar_t& swingLegLiftOff = 0.15,
			const scalar_t& swingTimeScale = 1.0)

	: Base(),
	  feetZDirectionPlanner_(swingLegLiftOff, swingTimeScale)
	{}

	/**
	 * Copy constructor
	 */
	SwitchedModelLogicRulesBase(const SwitchedModelLogicRulesBase& rhs)

	: Base(rhs),
	  motionPhasesSequence_(rhs.motionPhasesSequence_),
	  feetZDirectionPlanner_(rhs.feetZDirectionPlanner_),
	  contactFlagsStock_(rhs.contactFlagsStock_),
	  zDirectionRefsStock_(rhs.zDirectionRefsStock_),
	  endEffectorStateConstraints_(rhs.endEffectorStateConstraints_)
	{}

	/**
	 * Destructor
	 */
	virtual ~SwitchedModelLogicRulesBase()
	{}

	/**
	 * Sets motion constraints which include gait sequence constraints and state-only constraint defined by gaps, etc.
	 *
	 * @param [in] switchingTimes: The switching times.
	 * @param [in] motionPhasesSequence: The sequence of the motion phase.
	 * @param [in] endEffectorStateConstraints: An array of the state-only constraint defined by gaps, etc.
	 */
	void setMotionConstraints(const scalar_array_t& switchingTimes,
			const size_array_t& motionPhasesSequence,
			const std::vector<EndEffectorConstraintBase::ConstPtr>& endEffectorStateConstraints = std::vector<EndEffectorConstraintBase::ConstPtr>()) {

		setGaitSequence(switchingTimes, motionPhasesSequence);

		endEffectorStateConstraints_ = endEffectorStateConstraints;
	}

	/**
	 * Sets gait sequence.
	 *
	 * @param [in] switchingTimes: The switching times.
	 * @param [in] motionPhasesSequence: The sequence of the motion phase.
	 */
	void setGaitSequence(const scalar_array_t& switchingTimes,
			const size_array_t& motionPhasesSequence) {

		Base::switchingTimes_ = switchingTimes;
		motionPhasesSequence_ = motionPhasesSequence;
		size_t numSubsystems = motionPhasesSequence.size();

		if (switchingTimes.size()+1 != numSubsystems)
			throw std::runtime_error("The number of motion phases should be 1 plus number of the switching times.");

		contactFlagsStock_.resize(numSubsystems);
		zDirectionRefsStock_.resize(numSubsystems);
		for (size_t i=0; i<numSubsystems; i++) {
			contactFlagsStock_[i] = modeNumber2StanceLeg(motionPhasesSequence[i]);
			feetZDirectionPlanner_.planSingleMode(i, motionPhasesSequence, switchingTimes, zDirectionRefsStock_[i]);
		}
	}


	/**
	 * Retrieves the contact flags of the requested motion phase.
	 *
	 * @param [in] index: The requested motion phase.
	 * @param [out] contactFlags: A contact flags array which determines the stance legs.
	 */
	void getContactFlags(const size_t& index,
			contact_flags_t& contactFlags) const {

		if (index >= contactFlagsStock_.size())
			throw std::runtime_error("The requested index refers to an out-of-bound motion phase.");

		contactFlags = contactFlagsStock_[index];
	}


	/**
	 * Retrieves constraints information of the motion phase specified by th given index. This information
	 * is sufficient to determine the active constraints of the requested motion phase.
	 *
	 * @param [in] index: The requested motion phase.
	 * @param [out] contactFlags: A contact flags array which determines the stance legs.
	 * @param [out] zDirectionRefs: An array of z-direction motion reference for the swing legs (note that
	 * for the stance legs the behavior of the reference generator is not defined).
	 */
	void getMotionPhaseLogics(const size_t& index,
			contact_flags_t& contactFlags,
			const feet_cpg_ptr_t* zDirectionRefsPtr) const {

		if (index >= contactFlagsStock_.size())
			throw std::runtime_error("The requested index refers to an out-of-bound motion phase.");

		contactFlags = contactFlagsStock_[index];

		zDirectionRefsPtr = &zDirectionRefsStock_[index];
	}

	/**
	 * Get a const pointer to state-only constraints for end-effectors.
	 *
	 * @return A const pointer to the array of const pointer for endEffectorStateConstraint objects.
	 */
	const std::vector<EndEffectorConstraintBase::ConstPtr>* getEndEffectorStateConstraintsPtr() const {

		return &endEffectorStateConstraints_;
	}

	/**
	 * Adjust the controller based on the last changes in the logic rules.
	 *
	 * @param controller: The controller which will be modified.
	 */
	virtual void adjustController(controller_t& controller) const override {

		// TODO
	}

private:
	size_array_t motionPhasesSequence_;
	feet_z_planner_t feetZDirectionPlanner_;

	std::vector<contact_flags_t> contactFlagsStock_;
	std::vector<feet_cpg_ptr_t> zDirectionRefsStock_;

	std::vector<EndEffectorConstraintBase::ConstPtr> endEffectorStateConstraints_;

};

} // namespace switched_model

#endif /* SWITCHEDMODELLOGICRULESBASE_H_ */
