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

#ifndef MRT_ROS_DUMMY_CARTPOLE_OCS2_H_
#define MRT_ROS_DUMMY_CARTPOLE_OCS2_H_

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>
#include <ocs2_robotic_examples/examples/cart_pole/CartpoleVisualizer.hpp>
#include <ocs2_robotic_examples/examples/cart_pole/definitions.h>

namespace ocs2 {
namespace cartpole {

class MRT_ROS_Dummy_Cartpole : public MRT_ROS_Dummy_Loop<cartpole::STATE_DIM_, cartpole::INPUT_DIM_, NullLogicRules< cartpole::STATE_DIM_, cartpole::INPUT_DIM_>> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef NullLogicRules<cartpole::STATE_DIM_, cartpole::INPUT_DIM_> logic_rules_t;
	typedef MRT_ROS_Dummy_Loop<cartpole::STATE_DIM_, cartpole::INPUT_DIM_, logic_rules_t> BASE;

	/**
	 * Constructor.
	 *
	 * @param [in] mrtPtr: A pointer to MRT.
	 * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
	 * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
	 * will be simulated to run by this frequency. Note that this might not be the MPC's realtime frequency.
	 */
	MRT_ROS_Dummy_Cartpole(
			const mrt_ptr_t& mrtPtr,
			const scalar_t& mrtDesiredFrequency,
			const scalar_t& mpcDesiredFrequency)
	: BASE(mrtPtr, mrtDesiredFrequency, mpcDesiredFrequency),
	  visualizer_()
	{
		visualizer_.start();
	}

	/**
	 * Destructor.
	 */
	virtual ~MRT_ROS_Dummy_Cartpole() = default;

	/**
	 * The initialization of the observation
	 *
	 * @param [in] initObservation: The initial observation.
	 */
	virtual void init(const system_observation_t& initObservation) {
		BASE::init(initObservation);
	}

protected:
	/**
	 * Visualizes the current observation.
	 *
	 * @param [in] observation: The current observation.
	 */
	virtual void publishVisualizer(const system_observation_t& observation) override {
		visualizer_.drawWorld(observation.state()(1), observation.state()(0), observation.input()(0));
	}

private:
	CartpoleVisualizer visualizer_;

};

} // namespace cartpole
} // namespace ocs2

#endif /* MRT_ROS_DUMMY_CARTPOLE_OCS2_H_ */
