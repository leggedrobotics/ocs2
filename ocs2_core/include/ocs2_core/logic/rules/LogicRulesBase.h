/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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
class LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<0, 0> DIMENSIONS;
	using size_array_t = typename DIMENSIONS::size_array_t;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;

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
	 
	= default;

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
	 * Retrieves the number of the event times.
	 *
	 * @return number of event times.
	 */
	size_t getNumEventTimes() const {

		return eventTimes_.size();
	}

	/**
	 * Displays event information.
	 */
	virtual void display() const {

		std::cerr << std::endl << "Event Times:\n\t {";
		for (auto& t: eventTimes_) {
			std::cerr << t << ", ";
		}
		if (!eventTimes_.empty()) {  std::cerr << "\b\b";
		}
		std::cerr << "}" << std::endl;

		std::cerr << "Event counters:\n\t {";
		for (size_t i=0; i<=eventTimes_.size() ;i++) {
			std::cerr << i << ", ";
		}
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
	 * This method can be used to update the internal variables. This method will be called by any
	 * program that tries to update the logic rules variables.
	 */
	virtual void update() = 0;

protected:

	scalar_array_t eventTimes_;

};


} // namespace ocs2

#endif /* OGICRULESBASE_OCS2_H_ */
