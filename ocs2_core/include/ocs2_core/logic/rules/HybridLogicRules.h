/*
 * HybridLogicRules.h
 *
 *  Created on: Mar 22, 2018
 *      Author: farbod
 */

#ifndef HYBRIDLOGICRULES_OCS2_H_
#define HYBRIDLOGICRULES_OCS2_H_

#include "ocs2_core/logic/rules/LogicRulesBase.h"

namespace ocs2{

/**
 * Mode sequence template.
 *
 */
template <typename scalar_t = double>
class ModeSequenceTemplate
{
public:
	ModeSequenceTemplate()
	: templateSwitchingTimes_(0),
	  templateSubsystemsSequence_(0)
	{}

	/**
	 * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration of the template logic. t_1 to t_n are the event moments.
	 */
	std::vector<scalar_t> templateSwitchingTimes_;

	/**
	 * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is active in period [t_i, t_(i+1)]
	 */
	std::vector<size_t> templateSubsystemsSequence_;

	/**
	 * Displays template information.
	 */
	void display() const {

		std::cerr << std::endl << "Template switching times:\n\t {";
		for (auto& s: templateSwitchingTimes_)
			std::cerr << s << ", ";
		if (!templateSwitchingTimes_.empty())  std::cerr << "\b\b";
		std::cerr << "}" << std::endl;

		std::cerr << "Template subsystem sequence:\n\t {";
		for (auto& s: templateSubsystemsSequence_)
			std::cerr << s << ", ";
		if (!templateSubsystemsSequence_.empty())  std::cerr << "\b\b";
		std::cerr << "}" << std::endl;
	}
};

/**
 * Logic rules base class
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class HybridLogicRules : public LogicRulesBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase<STATE_DIM, INPUT_DIM> BASE;

	typedef typename BASE::size_array_t 		size_array_t;
	typedef typename BASE::scalar_t 			scalar_t;
	typedef typename BASE::scalar_array_t 		scalar_array_t;
	typedef typename BASE::controller_t 		controller_t;
	typedef typename BASE::controller_array_t 	controller_array_t;

	typedef ModeSequenceTemplate<scalar_t> logic_template_type;

	/**
	 * Default constructor
	 */
	HybridLogicRules()
	: BASE(),
	  subsystemsSequence_(1, 0)
	{}

	/**
	 * Constructor
	 */
	HybridLogicRules(
			const scalar_array_t& eventTimes,
			const size_array_t& subsystemsSequence)
	: BASE(eventTimes),
	  subsystemsSequence_(subsystemsSequence)
	{}

	/**
	 * Copy constructor
	 */
	HybridLogicRules(const HybridLogicRules& rhs)
	: BASE(rhs),
	  subsystemsSequence_(rhs.subsystemsSequence_)
	{}

	/**
	 * Destructor
	 */
	virtual ~HybridLogicRules() = default;

	/**
	 * Move assignment
	 */
	HybridLogicRules& operator=(HybridLogicRules&& other) = default;

	/**
	 * Assignment
	 */
	HybridLogicRules& operator=(const HybridLogicRules& other) = default;

	/**
	 * Displays event information.
	 */
	virtual void display() const override {

		BASE::display();

		std::cerr << "Subsystem sequence:\n\t {";
		for (auto& s: subsystemsSequence_)
			std::cerr << s << ", ";
		if (!subsystemsSequence_.empty())  std::cerr << "\b\b";
		std::cerr << "}" << std::endl;
	}

	/**
	 * Sets the mode sequence which include the sequence of the triggered subsystems and their respective
	 * transition time sequence.
	 * Note: The update method is called to update the derived class variables.
	 *
	 * @param [in] subsystemsSequence: The sequence of the triggered subsystems.
	 * @param [in] eventTimes: The sequence of the times in which mode transition took place.
	 */
	virtual void setModeSequence(
			const size_array_t& subsystemsSequence,
			const scalar_array_t& eventTimes) {

		if (subsystemsSequence.size() != eventTimes.size()+1)
			throw std::runtime_error("The number of subsystems should be 1 plus the number of the event times.");

		BASE::eventTimes_ = eventTimes;
		subsystemsSequence_ = subsystemsSequence;

		// update derived class internal variables
		update();
	}

	/**
	 * Retrieves the sequence of the triggered subsystems.
	 *
	 * @return A const reference to subsystemsSequence_.
	 */
	const size_array_t& subsystemsSequence() const {

		return subsystemsSequence_;
	}

	/**
	 * Retrieves the sequence of the triggered subsystems.
	 *
	 * @return A const reference to subsystemsSequence_.
	 */
	size_array_t& subsystemsSequence() {

		return subsystemsSequence_;
	}

	/**
	 * Used in the SLQ-MPC method to set the model sequence template.
	 *
	 * @param [in] modeSequenceTemplate: A dada type which includes all necessary information for modifying the logicRules.
	 */
	virtual void setModeSequenceTemplate(const logic_template_type& modeSequenceTemplate) = 0;

	/**
	 * Used in the SLQ-MPC method to inset a new user defined logic in the given time period.
	 * Note: use the update method to at the end to update your derived class variables
	 *
	 * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
	 * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
	 */
	virtual void insertModeSequenceTemplate(
			const scalar_t& startTime,
			const scalar_t& finalTime) = 0;

	/**
	 * Rewinds the class. This method is only called in the MPC class.
	 * Note: use the update method to at the end to update your derived class variables.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
	 * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
	 */
	virtual void rewind(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) = 0;

	/**
	 * Adjusts controller. This methos is called my the logicMachine whenever the logicRuls are updated.
	 * It allows the user to modify the controller to adapt to the changes of logics.
	 *
	 * @param controller: Control policy which should be adjusted.
	 */
	virtual void adjustController(controller_t& controller) const = 0;

	/**
	 * This method can be used to update the internal variables. This method should be called by any
	 * program that updates the logic rules variables e.g. rewind,
	 */
	virtual void update() = 0;


protected:
	size_array_t subsystemsSequence_;

};

} // namespace ocs2

#endif /* HYBRIDLOGICRULES_OCS2_H_ */
