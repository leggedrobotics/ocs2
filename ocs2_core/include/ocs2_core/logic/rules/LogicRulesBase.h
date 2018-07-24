/*
 * LogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef OGICRULESBASE_OCS2_H_
#define OGICRULESBASE_OCS2_H_

#include <memory>
#include <iostream>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * Logic rules base class
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_t 			scalar_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_array_t 		scalar_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::size_array_t 		size_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_t 		controller_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_array_t 	controller_array_t;

	/**
	 * Default constructor
	 */
	LogicRulesBase()
	: eventTimes_(0)
	{}

	/**
	 * Constructor
	 */
	LogicRulesBase(const scalar_array_t& eventTimes)
	: eventTimes_(eventTimes)
	{}

	/**
	 * Copy constructor
	 */
	LogicRulesBase(const LogicRulesBase& rhs)
	: eventTimes_(rhs.eventTimes_)
	{}

	/**
	 * Destructor
	 */
	virtual ~LogicRulesBase() = default;

	/**
	 * Move assignment
	 */
	LogicRulesBase& operator=(LogicRulesBase&& other) = default;

	/**
	 * Assignment
	 */
	LogicRulesBase& operator=(const LogicRulesBase& other) = default;

	/**
	 * Sets the event times.
	 *
	 * @param [in] eventTimes: The new event times.
	 */
	void setEventTimes(const scalar_array_t& eventTimes) {

		eventTimes_ = eventTimes;

		// update derived class variables
		update();
	}

	/**
	 * Retrieves the event times.
	 *
	 * @return A constant reference to eventTimes_.
	 */
	const scalar_array_t& eventTimes() const {

		return eventTimes_;
	}

	/**
	 * Retrieves the event times.
	 *
	 * @return A constant reference to eventTimes_.
	 */
	scalar_array_t& eventTimes() {

		return eventTimes_;
	}

	/**
	 * Displays event information.
	 */
	virtual void display() const {

		std::cerr << std::endl << "Event Times:\n\t {";
		for (auto& t: eventTimes_)
			std::cerr << t << ", ";
		if (!eventTimes_.empty())  std::cerr << "\b\b";
		std::cerr << "}" << std::endl;

		std::cerr << "Event counters:\n\t {";
		for (size_t i=0; i<=eventTimes_.size() ;i++)
			std::cerr << i << ", ";
		std::cerr << "\b\b}" << std::endl;
	}

	/**
	 * Rewinds the class. This method is only called in the MPC class.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
	 * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
	 */
	virtual void rewind(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) = 0;

	/**
	 * Adjusts controller. This method is called my the logicMachine whenever the logicRuls are updated.
	 * It allows the user to modify the controller to adapt to the changes of logics.
	 *
	 * @param controller: Control policy which should be adjusted.
	 */
	virtual void adjustController(controller_t& controller) const = 0;

	/**
	 * This method can be used to update the internal variables. This method will be called by any
	 * program that tries to update the logic rules variables.
	 */
	virtual void update() = 0;

protected:

	scalar_array_t eventTimes_;

};


} // namespace ocs2

#endif /* OGICRULESBASE_OCS2_H_ */
