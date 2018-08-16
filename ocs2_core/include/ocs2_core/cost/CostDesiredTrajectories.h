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

#ifndef COSTDESIREDTRAJECTORIES_OCS2_H_
#define COSTDESIREDTRAJECTORIES_OCS2_H_

#include <vector>
#include <iomanip>
#include <Eigen/Dense>

#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2{


/**
 * This class is an interface class for the cost desired trajectories.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class CostDesiredTrajectories
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<SCALAR_T> scalar_array_t;
	typedef Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> dynamic_vector_t;
	typedef std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>> dynamic_vector_array_t;
	typedef EigenLinearInterpolation<dynamic_vector_t>  dynamic_linear_interpolation_t;

	CostDesiredTrajectories(
			const scalar_array_t& desiredTimeTrajectory = scalar_array_t(),
			const dynamic_vector_array_t& desiredStateTrajectory = dynamic_vector_array_t(),
			const dynamic_vector_array_t& desiredInputTrajectory = dynamic_vector_array_t())
	: desiredTimeTrajectory_(desiredTimeTrajectory)
	, desiredStateTrajectory_(desiredStateTrajectory)
	, desiredInputTrajectory_(desiredInputTrajectory)
	{}

	CostDesiredTrajectories(const size_t& trajectorySize)
	: desiredTimeTrajectory_(trajectorySize)
	, desiredStateTrajectory_(trajectorySize)
	, desiredInputTrajectory_(trajectorySize)
	{}

	~CostDesiredTrajectories() = default;

	bool empty() const {

		return desiredTimeTrajectory_.empty();
	}

	void clear() {
		desiredTimeTrajectory_.clear();
		desiredStateTrajectory_.clear();
		desiredInputTrajectory_.clear();
	}

	void swap(CostDesiredTrajectories<SCALAR_T>& other) {

		desiredTimeTrajectory_.swap(other.desiredTimeTrajectory_);
		desiredStateTrajectory_.swap(other.desiredStateTrajectory_);
		desiredInputTrajectory_.swap(other.desiredInputTrajectory_);
	}

	scalar_array_t& desiredTimeTrajectory() { return desiredTimeTrajectory_; }
	const scalar_array_t& desiredTimeTrajectory() const { return desiredTimeTrajectory_; }

	dynamic_vector_array_t& desiredStateTrajectory() { return desiredStateTrajectory_; }
	const dynamic_vector_array_t& desiredStateTrajectory() const { return desiredStateTrajectory_; }

	dynamic_vector_array_t& desiredInputTrajectory() { return desiredInputTrajectory_; }
	const dynamic_vector_array_t& desiredInputTrajectory() const { return desiredInputTrajectory_; }

	void getDesiredStateFunc(
			dynamic_linear_interpolation_t& desiredStateFunc) const {

		if (desiredStateTrajectory_.empty()==false) {
			desiredStateFunc.reset();
			desiredStateFunc.setTimeStamp(&desiredTimeTrajectory_);
			desiredStateFunc.setData(&desiredStateTrajectory_);
		} else {
			desiredStateFunc.setZero();
		}
	}

	void getDesiredInputFunc(
			dynamic_linear_interpolation_t& desiredInputFunc) const {

		if (desiredInputTrajectory_.empty()==false) {
			desiredInputFunc.reset();
			desiredInputFunc.setTimeStamp(&desiredTimeTrajectory_);
			desiredInputFunc.setData(&desiredInputTrajectory_);
		} else {
			desiredInputFunc.setZero();
		}
	}

	void display() const {

		const int dispPrecision = 4;

		size_t N = desiredTimeTrajectory_.size();
		for (size_t i=0; i<N; i++) {

			std::cerr << "time: " << std::setprecision(dispPrecision) <<
					desiredTimeTrajectory_[i] << ",  " << std::endl;

			// state
			std::cerr << "state: [";
			for (size_t j=0; j<desiredStateTrajectory_[i].size(); j++) {
				std::cerr << std::setprecision(dispPrecision) <<
						desiredStateTrajectory_[i](j) << ",  ";
			}
			if (desiredStateTrajectory_[i].size()>0)
				std::cerr << "\b\b]" << ",  " << std::endl;
			else
				std::cerr << " ]" << ",  " << std::endl;

			// input
			std::cerr << "input: [";
			for (size_t j=0; j<desiredInputTrajectory_[i].size(); j++) {
				std::cerr << std::setprecision(dispPrecision) <<
						desiredInputTrajectory_[i](j) << ",  ";
			}
			if (desiredInputTrajectory_[i].size()>0)
				std::cerr << "\b\b]" << std::endl;
			else
				std::cerr << " ]" << std::endl;

		}  // end of i loop
	}


private:
	scalar_array_t 			desiredTimeTrajectory_;
	dynamic_vector_array_t 	desiredStateTrajectory_;
	dynamic_vector_array_t  desiredInputTrajectory_;
};

} // namespace ocs2

#endif /* COSTDESIREDTRAJECTORIES_OCS2_H_ */
