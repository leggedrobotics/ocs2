#include "ocs2_core/Dimensions.h"
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>
#include <ocs2_core/control/LinearController.h>
#include "Dynamics_StateRollOut.h"


#include <gtest/gtest.h>

using DIMENSIONS = ocs2::Dimensions<2, 1>;
using controller_t = ocs2::ControllerBase<2, 1>;

using scalar_t = typename DIMENSIONS::scalar_t;
using state_vector_t = typename DIMENSIONS::state_vector_t;
using time_interval_t = std::pair<scalar_t, scalar_t>;
using input_vector_t = typename DIMENSIONS::input_vector_t;
using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;

using scalar_array_t = typename DIMENSIONS::scalar_array_t;
using size_array_t = typename DIMENSIONS::size_array_t;
using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;

using time_interval_array_t = std::vector<time_interval_t>;

void testfunc(controller_t* input);

TEST(StateRolloutTests, Case1)
{
// Construct State TriggerdRollout Object
	ocs2::Rollout_Settings sets;
	ocs2::ball_tester_dyn dynamics;
	ocs2::StateTriggeredRollout<2,1> Rollout(dynamics,sets);
// Construct Variables for run
	// Simulation time
	scalar_t t0 = 0;
	scalar_t t1 = 100;
	// Initial State
	state_vector_t initState(2,0);
	initState[0] = 1;
	// Event times (none)
	scalar_array_t eventTimes(1,t0);
	// Controller (time constant zero controller)
	scalar_array_t timestamp(1,t0);

	input_vector_t bias;
	bias << 0 ;
	input_vector_array_t bias_array(1,bias);

	input_state_matrix_t gain;
	gain<< 0,0;
	input_state_matrix_array_t gain_array(1,gain);
	ocs2::LinearController<2,1> Control(timestamp,bias_array,gain_array);
	ocs2::LinearController<2,1>* Controller = &Control;

	// Trajectory storage
	scalar_array_t timeTrajectory(0);
	size_array_t eventsPastTheEndIndeces(0);
	state_vector_array_t stateTrajectory(0);
	input_vector_array_t inputTrajectory(0);
// Output State
	state_vector_t FinalState;
// Run
	FinalState = Rollout.run( t0,initState,t1,Controller,eventTimes,
	    				      timeTrajectory,eventsPastTheEndIndeces,
							  stateTrajectory,inputTrajectory);

	for(int i = 0; i<timeTrajectory.size();i++){
		// Test 1: Energy Conservation
		double energy = 9.81*stateTrajectory[i][0] + 0.5*stateTrajectory[i][1]*stateTrajectory[i][1];
		EXPECT_LT(std::fabs(energy - 9.81*stateTrajectory[0][0] + 0.5*stateTrajectory[0][1]*stateTrajectory[0][1]), 1e-6);
		// Test 2: No Significant penetration of Guard Surface
		EXPECT_GT(stateTrajectory[i][0], -1e-6);
		// Test 2b: No significant penetration of second Guard Surface (only checked on parts after initial conditions passes trough)
		if (timeTrajectory[i] > 0.45)
		{
		EXPECT_GT(-stateTrajectory[i][0] + 0.1 + timeTrajectory[i]/50, -1e-6);
		}
		// Optional output of state and time trajectories
		if(false)
		{
			std::cout<<i<<";"<<timeTrajectory[i]<<";"<<stateTrajectory[i][0]<<";"<<stateTrajectory	[i][1]<<std::endl;
		}
	}
}




int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

