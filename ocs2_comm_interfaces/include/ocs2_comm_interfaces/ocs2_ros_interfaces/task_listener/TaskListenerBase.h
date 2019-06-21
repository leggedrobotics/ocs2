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

#ifndef TASK_LISTENER_BASE_OCS2_H_
#define TASK_LISTENER_BASE_OCS2_H_

#include <memory>
#include <Eigen/StdVector>
#include <vector>
#include <ros/ros.h>


namespace ocs2{

/**
 * The class task listener interface class.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T=float>
class TaskListenerBase
{
public:
	typedef SCALAR_T scalar_t;

	typedef std::shared_ptr<TaskListenerBase<SCALAR_T>> shared_ptr_t;
	typedef std::vector<shared_ptr_t> shared_ptr_array_t;

	/**
	 * Default constructor.
	 *
	 */
	TaskListenerBase() = default;

	/**
	 * Default destructor.
	 */
	virtual ~TaskListenerBase() = default;

	/**
	 * This method should swap the active variables with the buffer variables.
	 */
	virtual void update() = 0;

	/**
	 * Gets a ROS node handle to for subscribing a topic.
	 */
	virtual void subscribe(::ros::NodeHandle& nodeHandle) = 0;

};

} // namespace ocs2


#endif /* TASK_LISTENER_BASE_OCS2_H_ */
