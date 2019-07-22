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

#ifndef MPC_ROS_CARTPOLE_OCS2_H_
#define MPC_ROS_CARTPOLE_OCS2_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include "ocs2_cart_pole_example//definitions.h"

namespace ocs2 {
namespace cartpole {

class MPC_ROS_Cartpole : public MPC_ROS_Interface<cartpole::STATE_DIM_, cartpole::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MPC_ROS_Interface<cartpole::STATE_DIM_, cartpole::INPUT_DIM_> BASE;

	typedef typename mpc_t::scalar_t scalar_t;
	typedef typename mpc_t::scalar_array_t scalar_array_t;
	typedef typename mpc_t::size_array_t size_array_t;
	typedef typename mpc_t::state_vector_t state_vector_t;
	typedef typename mpc_t::state_vector_array_t state_vector_array_t;
	typedef typename mpc_t::state_vector_array2_t state_vector_array2_t;
	typedef typename mpc_t::input_vector_t input_vector_t;
	typedef typename mpc_t::input_vector_array_t input_vector_array_t;
	typedef typename mpc_t::input_vector_array2_t input_vector_array2_t;
	typedef typename mpc_t::controller_t controller_t;
	typedef typename mpc_t::input_state_matrix_t input_state_matrix_t;
	typedef typename mpc_t::input_state_matrix_array_t input_state_matrix_array_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	typedef SystemObservation<cartpole::STATE_DIM_, cartpole::INPUT_DIM_> system_observation_t;

	typedef RosMsgConversions<cartpole::STATE_DIM_, cartpole::INPUT_DIM_> ros_msg_conversions_t;

	/**
	 * Default constructor
	 */
	MPC_ROS_Cartpole() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpcPtr: The MPC object to be interfaced.
	 * @param [in] nodeName: The node's name.
	 */
	MPC_ROS_Cartpole(
			mpc_t &mpc,
			const std::string &nodeName = "robot_mpc")
	: BASE(mpc, nodeName) {}

	/**
	 * Default destructor.
	 */
	virtual ~MPC_ROS_Cartpole() = default;
};

} // namespace cartpole
} // namespace ocs2

#endif /* MPC_ROS_CARTPOLE_OCS2_H_ */
