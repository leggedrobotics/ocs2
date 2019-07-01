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

#ifndef OCS2_OBSERVER_H_
#define OCS2_OBSERVER_H_

#include <string>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/integration/SystemEventHandler.h"
#include "ocs2_core/integration/ODE_Base.h"


namespace ocs2{

template <int STATE_DIM>
class IntegratorBase;

/**
 * Observer Class
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class Observer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using scalar_t = double;
	using scalar_array_t = std::vector<scalar_t>;
	typedef Eigen::Matrix<scalar_t,STATE_DIM,1> state_vector_t;
	typedef std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t>> state_vector_array_t;

    /**
     * Constructor
     * @param [in] eventHandler
     */
	Observer(const std::shared_ptr<SystemEventHandler<STATE_DIM> >& eventHandlerPtr = nullptr)

	: observeWrap([this](const state_vector_t& x, const scalar_t& t){ this->observe(x,t); }),
	  eventHandlerPtr_(eventHandlerPtr)
	{}

    /**
     * Observe function to retrieve the variable of interest.
     * @param [in] x: Current state.
     * @param [in] t: Current time.
     */
	void observe(const state_vector_t& x, const scalar_t& t) {

		// Store data
		stateTrajectoryPtr_->push_back(x);
		timeTrajectoryPtr_->push_back(t);

		// Check events
		if (eventHandlerPtr_ && eventHandlerPtr_->checkEvent(x, t)) {

			// Act on the event
			int eventID = eventHandlerPtr_->handleEvent(*stateTrajectoryPtr_, *timeTrajectoryPtr_);

			switch(eventID) {

			case sys_event_id::killIntegration:
			{
				throw std::runtime_error("Integration terminated due to an external signal triggered by a program.");
				break;
			}
			case sys_event_id::maxCall:
			{
				std::string msg = "Integration terminated since the maximum number of function calls is reached. ";
				msg += "State at termination time " + std::to_string(t) + ":\n [";
				for (size_t i=0; i<x.size()-1; i++) {  msg += std::to_string(x(i)) + ", ";
				}
				msg += std::to_string(x(x.size()-1)) + "]\n";
				throw std::runtime_error(msg);
				break;
			}
			default:
			{
				throw static_cast<size_t>(eventID);
			}
			}
		}

	}

	/**
	 * Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
	 */
	std::function<void (const state_vector_t& x, const scalar_t& t)> observeWrap;

	/**
	 * Set state trajectory pointer to observer.
	 *
	 * @param stateTrajectoryPtr
	 */
	void setStateTrajectory(state_vector_array_t* stateTrajectoryPtr) {
		stateTrajectoryPtr_ = stateTrajectoryPtr;
	}

	/**
	 * Set time trajectory pointer to observer.
	 *
	 * @param timeTrajectoryPtr
	 */
	void setTimeTrajectory(scalar_array_t* timeTrajectoryPtr) {
		timeTrajectoryPtr_ = timeTrajectoryPtr;
	}


private:
	std::shared_ptr<SystemEventHandler<STATE_DIM> > eventHandlerPtr_;

	scalar_array_t* timeTrajectoryPtr_;
	state_vector_array_t* stateTrajectoryPtr_;
};


} // namespace ocs2

#endif /* OCS2OBSERVER_H_ */
