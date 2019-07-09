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

#ifndef TIMEHORIZONESTIMATORBASE_H_
#define TIMEHORIZONESTIMATORBASE_H_

#include <memory>
#include <Eigen/Dense>

namespace ocs2 {

/**
 *
 * @tparam STATE_DIM
 */
template<size_t STATE_DIM>
class TimeHorizonEstimatorBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Ptr = std::shared_ptr<TimeHorizonEstimatorBase<STATE_DIM> >;

	TimeHorizonEstimatorBase() :
		currentTimeHorizon_(1000) {}

	virtual ~TimeHorizonEstimatorBase() = default;

	/**
	 * Updates the time horizon
	 * @param [in] currentInitialState
	 */
	virtual void updateTimeHorizon(
			const Eigen::Matrix<double, STATE_DIM, 1> &currentInitialState) { currentTimeHorizon_ = 0.0; };

	/**
	 * Gets time horizon
	 * @return double
	 */
	const double getTimeHorizon() const { return currentTimeHorizon_; }

protected:
	double currentTimeHorizon_;
};


}  // namespace ocs2


#endif /* TIMEHORIZONESTIMATORBASE_H_ */
