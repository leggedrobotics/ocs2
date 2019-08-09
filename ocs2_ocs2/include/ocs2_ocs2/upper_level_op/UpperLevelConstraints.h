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

#ifndef UPPER_LEVEL_CONSTRAINTS_OCS2_H_
#define UPPER_LEVEL_CONSTRAINTS_OCS2_H_

#include <ocs2_frank_wolfe/NLP_Constraints.h>

namespace ocs2 {

/**
 * This class is an interface to a NLP constraints.
 */
class UpperLevelConstraints final : public NLP_Constraints
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NLP_Constraints BASE;
	using scalar_t = typename NLP_Constraints::scalar_t;
	using scalar_array_t = typename NLP_Constraints::scalar_array_t;
	using dynamic_vector_t = typename NLP_Constraints::dynamic_vector_t;
	using dynamic_matrix_t = typename NLP_Constraints::dynamic_matrix_t;

	/**
	 * Default constructor.
	 */
	UpperLevelConstraints() = default;

	/**
	 * Default destructor.
	 */
	~UpperLevelConstraints() = default;

	/**
	 * Sets the initial time, and final time.
	 *
	 * @param [in] initTime: The initial time.
	 * @param [in] finalTime: The final time.
	 */
	void set(const scalar_t& initTime,
			const scalar_t& finalTime) {

		initTime_ = initTime;
		finalTime_ = finalTime;
	}

	void setCurrentParameter(const dynamic_vector_t& x) override {

		const size_t numEventTimes = x.size();

		eventTime_ = x;

		Cm_ = dynamic_matrix_t::Zero(numEventTimes+1, numEventTimes);
		for (size_t i=0; i<numEventTimes+1; i++) {

			if (i < numEventTimes)
				Cm_(i,i)  = +1.0;

			if (i > 0)
				Cm_(i,i-1) = -1.0;
		}

		Dv_ = dynamic_vector_t::Zero(numEventTimes+1);
		Dv_(0) = -initTime_;
		Dv_(numEventTimes) = finalTime_;
	}

	void getLinearInequalityConstraint(dynamic_vector_t& h) override {

		h = Cm_ * eventTime_ + Dv_;
	}

	void getLinearInequalityConstraintDerivative(dynamic_matrix_t& dhdx) override {

		dhdx = Cm_;
	}

private:
	scalar_t initTime_;
	scalar_t finalTime_;

	dynamic_vector_t eventTime_;

	dynamic_matrix_t Cm_;
	dynamic_vector_t Dv_;
};

}  // namespace ocs2

#endif /* UPPER_LEVEL_CONSTRAINTS_OCS2_H_ */
