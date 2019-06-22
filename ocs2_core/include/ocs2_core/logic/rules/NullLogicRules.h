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

#ifndef NULLLOGICRULES_OCS2_H_
#define NULLLOGICRULES_OCS2_H_

#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/rules/HybridLogicRules.h"

namespace ocs2{

/**
 * Null logic rules class.
 */
class NullLogicRules : public HybridLogicRules
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = HybridLogicRules;

	using size_array_t = BASE::size_array_t;
	using scalar_t = BASE::scalar_t;
	using scalar_array_t = BASE::scalar_array_t;
	using logic_template_type = BASE::logic_template_type;

	/**
	 * Constructor
	 */
	NullLogicRules()
	: BASE()
	{}

	/**
	 * Copy constructor
	 */
	NullLogicRules(const NullLogicRules& rhs) = default;

	/**
	 * Destructor
	 */
	~NullLogicRules() = default;

	/**
	 * Move assignment
	 */
	NullLogicRules& operator=(NullLogicRules&& other) = default;

	/**
	 * Assignment
	 */
	NullLogicRules& operator=(const NullLogicRules& other) = default;

	/**
	 * Rewinds the class. This method is only called in the MPC class.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
	 * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
	 */
	virtual void rewind(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) final
	{}

	/**
	 * This method can be used to update the internal variables. This method will be called by any
	 * program that tries to update the logic rules variables.
	 */
	void update() final
	{}

protected:
	/**
	 * Used in the SLQ-MPC method to insert a new user defined logic in the given time period.
	 * Note: use the update method to at the end to update your derived class variables
	 *
	 * @param [in] modeSequenceTemplate: A data type which includes all necessary information for modifying the logicRules.
	 * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
	 * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
	 */
	void insertModeSequenceTemplate(
			const logic_template_type& modeSequenceTemplate,
			const scalar_t& startTime,
			const scalar_t& finalTime) final
	{}

private:

};

} // namespace ocs2

#endif /* NULLLOGICRULES_OCS2_H_ */
