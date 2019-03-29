/*
 * MRT_ROS_Dummy_Quadruped.h
 *
 *  Created on: May 28, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Dummy_Quadruped(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const scalar_t& mrtDesiredFrequency,
		const std::string& robotName /*= "robot"*/,
		const scalar_t& mpcDesiredFrequency /*= -1*/)

	: BASE(typename BASE::mrt_ptr_t(new mrt_t(ocs2QuadrupedInterfacePtr, robotName)), mrtDesiredFrequency, mpcDesiredFrequency)
	, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
	, quadrupedXppVisualizer_(ocs2QuadrupedInterfacePtr, robotName, true)
{

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Dummy_Quadruped(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const typename BASE::mrt_ptr_t mrtPtr,
		const scalar_t& mrtDesiredFrequency,
		const std::string& robotName /*= "robot"*/,
		const scalar_t& mpcDesiredFrequency /*= -1*/)

		: BASE(mrtPtr, mrtDesiredFrequency, mpcDesiredFrequency)
		, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
		, quadrupedXppVisualizer_(ocs2QuadrupedInterfacePtr, robotName, true)
{

}

/******************************************************************************************************/
/******************************************************************************************************/
/**********************************************************************************     ********************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::~MRT_ROS_Dummy_Quadruped() {
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::launchVisualizerNode(int argc, char* argv[]) {

		quadrupedXppVisualizer_.launchVisualizerNode(argc, argv);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishVisualizer(
		const system_observation_t& observation) {

		quadrupedXppVisualizer_.publishObservation(observation);
}


}  // end of namespace switched_model

