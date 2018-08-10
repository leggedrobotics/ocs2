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

#ifndef EVENTHANDLERBASE_OCS2_H_
#define EVENTHANDLERBASE_OCS2_H_

#include <exception>
#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * Event handler class for ode solvers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class EventHandlerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EventHandlerBase<STATE_DIM>> Ptr;

	typedef Dimensions<STATE_DIM, 0> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 				scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t			state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t	state_vector_array_t;
	typedef typename DIMENSIONS::dynamic_vector_t		dynamic_vector_t;

	/**
	 * Default constructor
	 */
	EventHandlerBase()
	: systemPtr_(nullptr)
	{}

	/**
	 * Default destructor
	 */
	virtual ~EventHandlerBase() = default;

	/**
	 * Sets a pointer to the system dynamics. This method is invoked by the integrator class in
	 * order to share integrator's system dynamics with eventHandler.
	 *
	 * @param systemPtr: shared pointer to the integrator's system dynamics.
	 */
	void setSystem(const std::shared_ptr<ODE_Base<STATE_DIM>>& systemPtr) {

		systemPtr_ = systemPtr;
	}

	/**
	 * Resets the class.
	 */
	virtual void reset() = 0;

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean: Whether an event is active.
	 */
	virtual bool checkEvent(
			const state_vector_t& state,
			const scalar_t& time) = 0;

	/**
	 * The operation should be performed if an event is activated. The method gets references to the time and state
	 * trajectories. The current time and state are the last elements of their respective container.
	 * The method should also return a "Non-Negative" ID which indicates the a unique ID for the active events.
	 * Note that will the negative return values are reserved to handle internal events for the program.
	 *
	 * @param [out] stateTrajectory: The state trajectory which contains the current state vector as its last element.
	 * @param [out] timeTrajectory: The time trajectory which contains the current time as its last element.
	 * @retune boolean: A non-negative unique ID for the active events.
	 */
	virtual int handleEvent(
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory) = 0;

protected:
	/**
	 * System dynamics used by integrator.
	 */
	std::shared_ptr<ODE_Base<STATE_DIM>> systemPtr_;

};


} // namespace ocs2

#endif /* EVENTHANDLERBASE_OCS2_H_ */
