/*
 * PlanningTest.cpp
 *
 *  Created on: Nov 19, 2017
 *      Author: farbod
 */

#include <array>
#include <memory>
#include <vector>
#include <csignal>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/Eigen.hpp>

#include <ocs2_core/misc/LoadConfigFile.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"


void mySigintHandler(int sig)  {
	ros::shutdown();
	exit(0);
}

int main( int argc, char* argv[] )
{
	typedef ocs2::Dimensions<12+12,12+12> dimension_t;

	enum {
		state_dim = 12+12,
		input_dim = 12+12,
		output_dim = state_dim,
	};

	Eigen::IOFormat CleanFmtDisplay = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");

	/******************************************************************************************************/
	/********  Loading settings data ********/
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFile;
	taskFile = std::string(argv[1]) + "/task.info";

	double slowdown, visTime;
	anymal::OCS2AnymalInterface::loadVisualizationSettings(taskFile, slowdown, visTime);

	// initial state of the switched system
	Eigen::Matrix<double,36,1> initHyQState;
	ocs2::LoadConfigFile::loadMatrix(taskFile, "initialHyQState", initHyQState);

	/******************************************************************************************************/
	anymal::OCS2AnymalInterface anymalOptimization(argv[1], initHyQState);

	auto start = std::chrono::steady_clock::now();

	// run SLQ
	anymalOptimization.runSLQP(0.0, initHyQState);
	// run OCS2
//	anymalOptimization.runOCS2(0.0, initHyQState);
	// run MPC
//	anymalOptimization.runMPC(0.0, initHyQState);

	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;

	std::cout<<"Elapsed time is :  " << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()
			<< " ms "<< std::endl;

	double costFunction, constriantISE;
	anymalOptimization.getCostFuntion(costFunction, constriantISE);

	// switching time
	std::vector<double> switchingTimes;
	anymalOptimization.getSwitchingTimes(switchingTimes);
	std::vector<double> initSwitchingTimes=switchingTimes;

	// controller
	dimension_t::controller_array_t controllersStock;
	anymalOptimization.getController(controllersStock);

	// rollout
	std::vector<dimension_t::scalar_array_t> timeTrajectoriesStock;
	dimension_t::state_vector_array2_t 		 stateTrajectoriesStock;
	dimension_t::control_vector_array2_t 	 inputTrajectoriesStock;
	anymalOptimization.getTrajectories(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);

	std::vector<std::array<bool,4> > stanceLegSequene;
	anymalOptimization.getStanceLegSequene(stanceLegSequene);

	std::vector<switched_model::EndEffectorConstraintBase::Ptr> gapIndicatorPtrs;
	anymalOptimization.getGapIndicatorPtrs(gapIndicatorPtrs);

	/******************************************************************************************************/
	std::cerr <<"\n======================================" << std::endl;
	std::cerr << "SLQP results: "<< std::endl;
	std::cerr << "The total cost: " << costFunction << std::endl;
	std::cerr << "The total ISE:  " << constriantISE << std::endl;

//	hyq::SwitchedModelKinematics switchedModelKinematics;
//
//	dimension_t::eigen_scalar_array_t eigenInitSwitchingTime(timeTrajectoriesStock.size()+1);
//	dimension_t::eigen_scalar_array_t eigenSwitchingTime(timeTrajectoriesStock.size()+1);
//	std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > eigenSubsystemsMode(timeTrajectoriesStock.size());
//	//
//	dimension_t::eigen_scalar_array_t   timeTrajectory;
//	dimension_t::state_vector_array_t   stateTrajectory;
//	dimension_t::control_vector_array_t inputTrajectory;
//	dimension_t::eigen_scalar_array_t   controllerTimeTrajectory;
//	typedef Eigen::Matrix<double,1,24*3> pos_com_gain_t;
//	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > posGainTrajectory;
//	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > jointGainTrajectory;
//	std::vector<Eigen::Matrix<double,12,1>, Eigen::aligned_allocator<Eigen::Matrix<double,12,1>> > feetPositionTrajectory;
//	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > copPositionErrorTrajectory;
//
//	dimension_t::eigen_scalar_array_t iterationCost, iterationISE1, ocs2IterationCost;
//	anymalOptimization.getIterationsLog(iterationCost, iterationISE1, ocs2IterationCost);
//
//	for (size_t i=0; i<timeTrajectoriesStock.size(); i++)  {
//
//		if (timeTrajectoriesStock[i].size() == 0)  continue;
//
//		eigenSubsystemsMode[i] << stanceLegSequene[i][0], stanceLegSequene[i][1],
//				stanceLegSequene[i][2], stanceLegSequene[i][3];
//
//		eigenInitSwitchingTime[i](0) = initSwitchingTimes[i];
//		eigenSwitchingTime[i](0) = switchingTimes[i];
//
//		for (size_t k=0; k<timeTrajectoriesStock[i].size(); k++)  {
//
//			timeTrajectory.push_back( (Eigen::VectorXd(1) << timeTrajectoriesStock[i][k]).finished() );
//			stateTrajectory.push_back(stateTrajectoriesStock[i][k]);
//			inputTrajectory.push_back(inputTrajectoriesStock[i][k]);
//
//			// calculate the Base pose and local velocities
//			hyq::CoMDynamics::joints_coordinate_t qJoints(stateTrajectoriesStock[i][k].tail<12>());
//			hyq::CoMDynamics::joints_coordinate_t dqJoints(inputTrajectoriesStock[i][k].tail<12>());
//			hyq::CoMDynamics::com_coordinate_t comPose(stateTrajectoriesStock[i][k].head<6>());
//			hyq::CoMDynamics::com_coordinate_t comLocalVelocities(stateTrajectoriesStock[i][k].segment<6>(6));
//			hyq::CoMDynamics::com_coordinate_t basePose;
//			hyq::CoMDynamics::CalculateBasePose(qJoints, comPose, basePose);
//			hyq::CoMDynamics::com_coordinate_t baseLocalVelocities;
//			hyq::CoMDynamics::CalculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);
//
//			// inputTrajectory: contact forces to Origin frame contact forces
//			Eigen::Matrix3d o_R_b = hyq::SwitchedModelKinematics::RotationMatrixOrigintoBase(basePose.head<3>()).transpose();
//			for (size_t j=0; j<4; j++) {
//				Eigen::Vector3d lambda = inputTrajectoriesStock[i][k].segment<3>(3*j);
//				inputTrajectory.back().segment<3>(3*j) = o_R_b * lambda;
//			}
//
//			// calculate the feet positions
//			hyq::SwitchedModelKinematics::generalized_coordinate_t generalizedCoordinate;
//			generalizedCoordinate << basePose, qJoints;
//			std::array<Eigen::Vector3d,4> feetPositions;
//			switchedModelKinematics.feetPositionsOriginFrame(generalizedCoordinate, feetPositions);
//			Eigen::Matrix<double,12,1> feetPositionsVector;
//			for (size_t j=0; j<4; j++)  feetPositionsVector.segment<3>(3*j) = feetPositions[j];
//			feetPositionTrajectory.push_back(feetPositionsVector);
//
//			//
//			copPositionErrorTrajectory.push_back(copError(stanceLegSequene[i], qJoints, inputTrajectoriesStock[i][k].head<12>()));
//
//		}  // end of k loop
//
//		for (size_t k=0; k<controllersStock[i].time_.size(); k++) {
//			controllerTimeTrajectory.push_back( (Eigen::VectorXd(1) << controllersStock[i].time_[k]).finished() );
//
//			// CoM equivalent gains
//			Eigen::Matrix<double,3,24> ktemp;
//			ktemp.setZero();
//			for (size_t j=0; j<4; j++)
//				if (stanceLegSequene[i][j]==true)
//					ktemp += controllersStock[i].k_[k].block<3,24>(3*j,0) / 73.172;
//
//			pos_com_gain_t comLinearGains;
//			pos_com_gain_t jointGains;
//			for (size_t j=0; j<3; j++) {
//				comLinearGains.segment<24>(24*j) = ktemp.row(j);
//				jointGains.segment<24>(24*j) = controllersStock[i].k_[k].row(12+j);
//			}
//
//			posGainTrajectory.push_back(comLinearGains);
//			jointGainTrajectory.push_back(jointGains);
//		}  // end of k loop
//
//	}  // end of i loop
//
//	eigenInitSwitchingTime.back()(0) = initSwitchingTimes.back();
//	eigenSwitchingTime.back()(0) = switchingTimes.back();
//
//	/******************************************************************************************************/
//	std::string resultDir = std::string(argv[1])+"/result";
//	std::string modeFileName  = resultDir + "/subsystemsMode.xml";
//	std::string initSwitchingTimeFileName  = resultDir + "/initSwitchingTime.xml";
//	std::string switchingTimeFileName   = resultDir + "/switchingTime.xml";
//	std::string timeTrajectoryFileName  = resultDir + "/timeTrajectory.xml";
//	std::string stateTrajectoryFileName = resultDir + "/stateTrajectory.xml";
//	std::string inputTrajectoryFileName = resultDir + "/inputTrajectory.xml";
//	std::string controllerTimeTrajectoryFileName  = resultDir + "/controllerTimeTrajectory.xml";
//	std::string posGainTrajectoryFileName  = resultDir + "/posGainTrajectory.xml";
//	std::string jointGainTrajectoryFileName  = resultDir + "/jointGainTrajectory.xml";
//	std::string feetPositionTrajectoryFileName  = resultDir + "/feetPositionTrajectory.xml";
//	std::string copPositionErrorTrajectoryFileName  = resultDir + "/copPositionErrorTrajectory.xml";
//	std::string costFileName = resultDir + "/cost.xml";
//	std::string ise1FileName = resultDir + "/ise1.xml";
//	std::string costOcs2FileName = resultDir + "/ocs2Cost.xml";
//
//	{ // we need these brackets to make sure the archive goes out of scope and flushes
//		std::ofstream modeFileFile(modeFileName);
//		cereal::XMLOutputArchive modeFile_xmlArchive(modeFileFile);
//		modeFile_xmlArchive(CEREAL_NVP(eigenSubsystemsMode));
//
//		std::ofstream initSwitchingTimeFile(initSwitchingTimeFileName);
//		cereal::XMLOutputArchive initSwitchingTime_xmlArchive(initSwitchingTimeFile);
//		initSwitchingTime_xmlArchive(CEREAL_NVP(eigenInitSwitchingTime));
//
//		std::ofstream switchingTimeFile(switchingTimeFileName);
//		cereal::XMLOutputArchive switchingTime_xmlArchive(switchingTimeFile);
//		switchingTime_xmlArchive(CEREAL_NVP(eigenSwitchingTime));
//
//		std::ofstream timeTrajectoryFile(timeTrajectoryFileName);
//		cereal::XMLOutputArchive timeTrajectory_xmlArchive(timeTrajectoryFile);
//		timeTrajectory_xmlArchive(CEREAL_NVP(timeTrajectory));
//
//		std::ofstream stateTrajectoryFile(stateTrajectoryFileName);
//		cereal::XMLOutputArchive stateTrajectory_xmlArchive(stateTrajectoryFile);
//		stateTrajectory_xmlArchive(CEREAL_NVP(stateTrajectory));
//
//		std::ofstream inputTrajectoryFile(inputTrajectoryFileName);
//		cereal::XMLOutputArchive inputTrajectory_xmlArchive(inputTrajectoryFile);
//		inputTrajectory_xmlArchive(CEREAL_NVP(inputTrajectory));
//
//		std::ofstream controllerTimeTrajectoryFile(controllerTimeTrajectoryFileName);
//		cereal::XMLOutputArchive controllerTimeTrajectory_xmlArchive(controllerTimeTrajectoryFile);
//		controllerTimeTrajectory_xmlArchive(CEREAL_NVP(controllerTimeTrajectory));
//
//		std::ofstream posGainTrajectoryFile(posGainTrajectoryFileName);
//		cereal::XMLOutputArchive posGainTrajectory_xmlArchive(posGainTrajectoryFile);
//		posGainTrajectory_xmlArchive(CEREAL_NVP(posGainTrajectory));
//
//		std::ofstream jointGainTrajectoryFile(jointGainTrajectoryFileName);
//		cereal::XMLOutputArchive jointGainTrajectory_xmlArchive(jointGainTrajectoryFile);
//		jointGainTrajectory_xmlArchive(CEREAL_NVP(jointGainTrajectory));
//
//		std::ofstream feetPositionTrajectoryFile(feetPositionTrajectoryFileName);
//		cereal::XMLOutputArchive feetPositionTrajectory_xmlArchive(feetPositionTrajectoryFile);
//		feetPositionTrajectory_xmlArchive(CEREAL_NVP(feetPositionTrajectory));
//
//		std::ofstream copPositionErrorTrajectoryFile(copPositionErrorTrajectoryFileName);
//		cereal::XMLOutputArchive copPositionErrorTrajectory_xmlArchive(copPositionErrorTrajectoryFile);
//		copPositionErrorTrajectory_xmlArchive(CEREAL_NVP(copPositionErrorTrajectory));
//
//		std::ofstream costFile(costFileName);
//		cereal::XMLOutputArchive costFile_xmlArchive(costFile);
//		costFile_xmlArchive(CEREAL_NVP(iterationCost));
//
//		std::ofstream ise1File(ise1FileName);
//		cereal::XMLOutputArchive ise1File_xmlArchive(ise1File);
//		ise1File_xmlArchive(CEREAL_NVP(iterationISE1));
//
//		std::ofstream costOcs2File(costOcs2FileName);
//		cereal::XMLOutputArchive costOcs2File_xmlArchive(costOcs2File);
//		costOcs2File_xmlArchive(CEREAL_NVP(ocs2IterationCost));
//	}

//	ros::init(argc, argv, "switched_state_publisher", ros::init_options::NoSigintHandler);
//	signal(SIGINT, mySigintHandler);
//	ros::NodeHandle n;
//	// publish the gaps
////	ros::Publisher gapPublisher = n.advertise<visualization_msgs::Marker>("gap", 100);
////	hyq::publishGapMessages(gapIndicatorPtrs, gapPublisher);
//	// publish hyq
////	ConvertToRosMsg<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>::TrajectoryMsg rosStateTrajectory;
//	msg_ocs2_hyq::HyqStateJointsTrajectory rosStateTrajectory;
//	ConvertToRosMsg<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>::ToTrajectoryMsg(
//			timeTrajectoriesStock, stateTrajectoriesStock, 0.02, rosStateTrajectory);
//	std::string trajectoryTopic;
//	ros::param::param<std::string>("trajectory_topic", trajectoryTopic, std::string("/hyqTrajectory"));
////	ros::Publisher publisher = n.advertise<hyqb_msgs::Trajectory>(trajectoryTopic, 10);
//	ros::Publisher publisher = n.advertise<msg_ocs2_hyq::HyqStateJointsTrajectory>("robot_trajectory_joints", 10);
//	while (ros::ok)  {
////		hyq::publishGapMessages(gapIndicatorPtrs, gapPublisher);
//
//		// send out the trajectory
//		publisher.publish(rosStateTrajectory);
//		ros::spinOnce();
//		sleep(2);
//		std::cerr << "visualizing ..." << std::endl;
//	}

}




