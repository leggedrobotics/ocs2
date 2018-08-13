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
#include <rosbag/bag.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/Eigen.hpp>

#include <ocs2_core/misc/loadEigenMatrix.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"
#include <pathfile.h>

//#include <gperftools/profiler.h>

void mySigintHandler(int sig)  {
	ros::shutdown();
	exit(0);
}

using namespace anymal;

int main( int argc, char* argv[] )
{
	typedef OCS2AnymalInterface ocs2_robot_interface_t;

	enum {
		state_dim = 12+12,
		input_dim = 12+12,
		output_dim = state_dim,
	};

	Eigen::IOFormat CleanFmtDisplay = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");

	/******************************************************************************************************/
	/********  Loading settings data ********/
	std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cout << "Current time: " << std::ctime(&currentDate);

	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFile = std::string(PACKAGE_PATH) + "/config/" + std::string(argv[1]) + "/task.info";
	std::cerr << "Loading task file: " << taskFile << std::endl;

	double slowdown, visTime;
	ocs2_robot_interface_t::loadVisualizationSettings(taskFile, slowdown, visTime);

	// initial state of the switched system
	ocs2_robot_interface_t::rbd_state_vector_t initialRbdState;
	ocs2::loadEigenMatrix(taskFile, "initialRobotState", initialRbdState);

	double dt, initTime, finalTime, initSettlingTime;
	ocs2_robot_interface_t::loadSimulationSettings(taskFile, dt, finalTime, initSettlingTime);
	initTime = initSettlingTime;

	/******************************************************************************************************/
	ocs2_robot_interface_t optimizationInterface(taskFile);

	auto start = std::chrono::steady_clock::now();

//	ProfilerStart("/home/farbod/Programs/anymal_ws/profiler_output");
	// run SLQ
	optimizationInterface.runSLQ(initTime, initialRbdState, finalTime);
	// run OCS2
//	optimizationInterface.runOCS2(0.0, initialRbdState);
	// run MPC
//	optimizationInterface.runMPC(0.0, initialRbdState);

//	ProfilerStop();

	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;

	std::cout<<"Elapsed time is :  "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()
			<< " ms "<< std::endl;

	double costFunction, constriantISE1, constriantISE2;
	optimizationInterface.getPerformanceIndeces(costFunction, constriantISE1, constriantISE2);

	// switching time
	const ocs2_robot_interface_t::scalar_array_t* eventTimesPtr;
	optimizationInterface.getEventTimesPtr(eventTimesPtr);

	// controller
	const ocs2_robot_interface_t::controller_array_t* controllersStockPtr;
	optimizationInterface.getOptimizedControllerPtr(controllersStockPtr);

	// rollout
	const std::vector<ocs2_robot_interface_t::scalar_array_t>* timeTrajectoriesStockPtr;
	const ocs2_robot_interface_t::state_vector_array2_t*	stateTrajectoriesStockPtr;
	const ocs2_robot_interface_t::input_vector_array2_t* 	inputTrajectoriesStockPtr;
	optimizationInterface.getOptimizedTrajectoriesPtr(timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr);

	const std::vector<ocs2_robot_interface_t::contact_flag_t>* contactFlagsSequencePtr;
	optimizationInterface.getContactFlagsSequencePtr(contactFlagsSequencePtr);

	std::vector<switched_model::EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs;
	optimizationInterface.getGapIndicatorPtrs(gapIndicatorPtrs);

	/******************************************************************************************************/
	std::cerr <<"\n======================================" << std::endl;
	std::cerr << "SLQ results: "<< std::endl;
	std::cerr << "The total cost: " << costFunction << std::endl;
	std::cerr << "The type-1 ISE: " << constriantISE1 << std::endl;
	std::cerr << "The type-2 ISE: " << constriantISE2 << std::endl;


	/******************************************************************************************************/
//	ocs2_robot_interface_t::state_vector_t initState = stateTrajectoriesStockPtr->front().front();
//	ocs2_robot_interface_t::scalar_array_t partitioningTimes = optimizationInterface.getSLQPtr()->getPartitioningTimes();
//
//	std::vector<ocs2_robot_interface_t::scalar_array_t> timeTrajectoriesStockST;
//	std::vector<ocs2_robot_interface_t::size_array_t> eventsPastTheEndIndecesStockST;
//	ocs2_robot_interface_t::state_vector_array2_t stateTrajectoriesStockST;
//	ocs2_robot_interface_t::input_vector_array2_t inputTrajectoriesStockST;
//
//	optimizationInterface.getSLQPtr()->rolloutStateTriggeredTrajectory(
//			initTime, initState, finalTime,
//			partitioningTimes, *controllersStockPtr,
//			timeTrajectoriesStockST, eventsPastTheEndIndecesStockST,
//			stateTrajectoriesStockST, inputTrajectoriesStockST);
//
//	for (size_t i=0; i<timeTrajectoriesStockPtr->size(); i++) {
//		std::cout << "\nPartition: " << i << std::endl;
//
//		std::cout << "time: " << timeTrajectoriesStockPtr->at(i).front() << std::endl;
//		std::cout << "State error norm: " <<
//				(stateTrajectoriesStockPtr->at(i).front() - stateTrajectoriesStockST.at(i).front()).norm() / stateTrajectoriesStockPtr->at(i).front().norm()  << std::endl;
//
//		std::cout << "time: " << timeTrajectoriesStockPtr->at(i).front() << std::endl;
//		std::cout << "Input error norm: " <<
//				(inputTrajectoriesStockPtr->at(i).front() - inputTrajectoriesStockST.at(i).front()).norm() / inputTrajectoriesStockPtr->at(i).front().norm()  << std::endl;
//
//		std::cout << "time: " << timeTrajectoriesStockPtr->at(i).back() << std::endl;
//		std::cout << "State error norm: " <<
//				(stateTrajectoriesStockPtr->at(i).back() - stateTrajectoriesStockST.at(i).back()).norm() / stateTrajectoriesStockPtr->at(i).back().norm()  << std::endl;
//
//		std::cout << "time: " << timeTrajectoriesStockPtr->at(i).back() << std::endl;
//		std::cout << "Input error norm: " <<
//				(inputTrajectoriesStockPtr->at(i).back() - inputTrajectoriesStockST.at(i).back()).norm() / inputTrajectoriesStockPtr->at(i).back().norm()  << std::endl;
//	}
//
//	ocs2::ModeSequenceTemplate<double> modeSequenceTemplate;
//	modeSequenceTemplate.templateSubsystemsSequence_ = std::vector<size_t>{11, 12, 13, 14};
//	modeSequenceTemplate.templateSwitchingTimes_ = std::vector<double>{0.0, 0.1, 0.6, 1.1, 1.6};
//	modeSequenceTemplate.display();
//
//	ocs2_robot_interface_t::logic_rules_t logicRules = static_cast<ocs2_robot_interface_t::logic_rules_t>(
//			optimizationInterface.getSLQPtr()->getLogicRules());
//	logicRules.display();
//
//	logicRules.setModeSequenceTemplate(1.0, 4.0, modeSequenceTemplate);
//	logicRules.display();
//
//	logicRules.rewind(1.9, 5.2);
//	logicRules.display();

	/******************************************************************************************************/

	AnymalKinematics switchedModelKinematics;
	AnymalCom comModel;

	ocs2_robot_interface_t::eigen_scalar_array_t eigenSwitchingTime(timeTrajectoriesStockPtr->size()+1);
//	std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > eigenSubsystemsMode(timeTrajectoriesStockPtr->size());
	//
	ocs2_robot_interface_t::eigen_scalar_array_t catTimeTrajectory;
	ocs2_robot_interface_t::state_vector_array_t catStateTrajectory;
	ocs2_robot_interface_t::input_vector_array_t catInputTrajectory;
	ocs2_robot_interface_t::eigen_scalar_array_t controllerTimeTrajectory;
	typedef Eigen::Matrix<double,1,24*3> pos_com_gain_t;
	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > posGainTrajectory;
	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > jointGainTrajectory;
	std::vector<Eigen::Matrix<double,12,1>, Eigen::aligned_allocator<Eigen::Matrix<double,12,1>> > feetPositionTrajectory;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > copPositionErrorTrajectory;

	ocs2_robot_interface_t::eigen_scalar_array_t iterationCost, iterationISE1, ocs2IterationCost;
	optimizationInterface.getIterationsLog(iterationCost, iterationISE1, ocs2IterationCost);

	// creates a bag file in current directory
	rosbag::Bag bag;
	bag.open("ocs2_anymal_traj.bag", rosbag::bagmode::Write);

	xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg;

	ros::Time::init();
	auto startTime = ros::Time::now();

	robotStateCartesianTrajectoryMsg.header.stamp = startTime;

	for (size_t i=0; i<timeTrajectoriesStockPtr->size(); i++)  {

		const ocs2_robot_interface_t::scalar_array_t& 			timeTrajectory  = timeTrajectoriesStockPtr->at(i);
		const ocs2_robot_interface_t::state_vector_array_t& 	stateTrajectory = stateTrajectoriesStockPtr->at(i);
		const ocs2_robot_interface_t::input_vector_array_t& 	inputTrajectory = inputTrajectoriesStockPtr->at(i);
		const ocs2_robot_interface_t::controller_t& 			controller      = controllersStockPtr->at(i);

		if (timeTrajectory.size() == 0)  continue;

//		eigenSubsystemsMode[i] << stanceLegSequene[i][0], stanceLegSequene[i][1],
//				stanceLegSequene[i][2], stanceLegSequene[i][3];

//		eigenSwitchingTime[i](0) = eventTimesPtr->at(i);

		for (size_t k=0; k<timeTrajectory.size(); k++)  {

			catTimeTrajectory.push_back( (Eigen::VectorXd(1) << timeTrajectory[k]).finished() );
			catStateTrajectory.push_back(stateTrajectory[k]);
			catInputTrajectory.push_back(inputTrajectory[k]);

			// calculate the Base pose and local velocities
			ocs2_robot_interface_t::joint_coordinate_t qJoints(stateTrajectory[k].tail<12>());
			ocs2_robot_interface_t::joint_coordinate_t dqJoints(inputTrajectory[k].tail<12>());
			ocs2_robot_interface_t::base_coordinate_t comPose(stateTrajectory[k].head<6>());
			ocs2_robot_interface_t::base_coordinate_t comLocalVelocities(stateTrajectory[k].segment<6>(6));
			ocs2_robot_interface_t::base_coordinate_t basePose;
			ocs2_robot_interface_t::base_coordinate_t baseLocalVelocities;
			comModel.calculateBasePose(qJoints, comPose, basePose);
			comModel.calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

			// inputTrajectory: contact forces to Origin frame contact forces
			Eigen::Matrix3d o_R_b = switched_model::RotationMatrixOrigintoBase(basePose.head<3>()).transpose();
			for (size_t j=0; j<4; j++) {
				Eigen::Vector3d lambda = inputTrajectory[k].segment<3>(3*j);
				catInputTrajectory.back().segment<3>(3*j) = o_R_b * lambda;
			}

			// calculate the feet positions
			AnymalKinematics::generalized_coordinate_t generalizedCoordinate;
			generalizedCoordinate << basePose, qJoints;
			std::array<Eigen::Vector3d,4> feetPositions;
			switchedModelKinematics.feetPositionsOriginFrame(generalizedCoordinate, feetPositions);
			Eigen::Matrix<double,12,1> feetPositionsVector;
			for (size_t j=0; j<4; j++)
			{
				feetPositionsVector.segment<3>(3*j) = feetPositions[j];
			}
			feetPositionTrajectory.push_back(feetPositionsVector);

//			// Cop error TODO
//			SwitchedModelCost anymalCost(stanceLegSequene[i], ...);
//			cost_funtion_t(initStanceLegSequene_[i], Q_, nondiagonalR,
//							desiredTimeTrajectoriesStock_[i], desiredStateTrajectoriesStock_[i], uNominalTrajectory, QFinal_, xFinal_, options_.copWeight_)
//
//			double copCost;											// dummy
//			Eigen::Matrix<double,12,1> devJoints_copCost;			// dummy
//			Eigen::Matrix<double,12,1> devLambda_copCost;			// dummy
//			Eigen::Matrix<double,12,12> hessJoints_copCost;			// dummy
//			Eigen::Matrix<double,12,12> hessLambda_copCost;			// dummy
//			Eigen::Matrix<double,12,12> devLambdaJoints_copCost;	// dummy
//			Eigen::Vector2d copError;
//			Eigen::Matrix<double,12,1> lambda = inputTrajectory.back(); //TODO is this correct??
//			anymalCost.copErrorCostFunc(qJoints, lambda, copCost, devJoints_copCost, devLambda_copCost,
//					hessJoints_copCost, hessLambda_copCost,
//					devLambdaJoints_copCost, copError);

			//construct the message
			xpp_msgs::RobotStateCartesian point;

			Eigen::Quaternion<double> qx( cos(basePose(0)/2),   sin(basePose(0)/2),   0.0,   0.0 );
			Eigen::Quaternion<double> qy( cos(basePose(1)/2),   0.0,   sin(basePose(1)/2),   0.0 );
			Eigen::Quaternion<double> qz( cos(basePose(2)/2),   0.0,   0.0,   sin(basePose(2)/2) );
			Eigen::Quaternion<double> qxyz = qz*qy*qx;
			point.base.pose.orientation.x = qxyz.x();
			point.base.pose.orientation.y = qxyz.y();
			point.base.pose.orientation.z = qxyz.z();
			point.base.pose.orientation.w = qxyz.w();
			point.base.pose.position.x = basePose(3);
			point.base.pose.position.y = basePose(4);
			point.base.pose.position.z = basePose(5);

			point.base.twist.linear.x  = baseLocalVelocities(0);
			point.base.twist.linear.y  = baseLocalVelocities(1);
			point.base.twist.linear.z  = baseLocalVelocities(2);
			point.base.twist.angular.x = baseLocalVelocities(3);
			point.base.twist.angular.y = baseLocalVelocities(4);
			point.base.twist.angular.z = baseLocalVelocities(5);

//			point.base.accel =

			point.time_from_start = ros::Duration(timeTrajectory[k]);

			constexpr int numEE = 4;
			point.ee_motion.resize(numEE);
			point.ee_forces.resize(numEE);
			point.ee_contact.resize(numEE);
			for(size_t ee_k=0; ee_k < numEE; ee_k++){
				point.ee_motion[ee_k].pos.x = feetPositionsVector(3*ee_k + 0);
				point.ee_motion[ee_k].pos.y = feetPositionsVector(3*ee_k + 1);
				point.ee_motion[ee_k].pos.z = feetPositionsVector(3*ee_k + 2);

//				point.ee_motion[ee_k].vel.x =
//				point.ee_motion[ee_k].acc.x =

				point.ee_forces[ee_k].x = inputTrajectory.back()(3*ee_k + 0);
				point.ee_forces[ee_k].y = inputTrajectory.back()(3*ee_k + 1);
				point.ee_forces[ee_k].z = inputTrajectory.back()(3*ee_k + 2);

//				point.ee_contact[ee_k] = stanceLegSequene[i][ee_k];
			}

			// const auto stamp = point.time_from_start + startTime;
			const auto stamp = ros::Time(startTime.toSec() + timeTrajectory[k]);

			bag.write("xpp/state_des",stamp, point);
			robotStateCartesianTrajectoryMsg.points.push_back(point);

		}  // end of k loop

		for (size_t k=0; k<controller.time_.size(); k++) {
			controllerTimeTrajectory.push_back( (Eigen::VectorXd(1) << controller.time_[k]).finished() );

			// CoM equivalent gains
			Eigen::Matrix<double,3,24> ktemp;
			ktemp.setZero();
//			for (size_t j=0; j<4; j++)
//				if (stanceLegSequene[i][j]==true)
//					ktemp += controller.k_[k].block<3,24>(3*j,0) / 73.172;

			pos_com_gain_t comLinearGains;
			pos_com_gain_t jointGains;
			for (size_t j=0; j<3; j++) {
				comLinearGains.segment<24>(24*j) = ktemp.row(j);
				jointGains.segment<24>(24*j) = controller.k_[k].row(12+j);
			}

			posGainTrajectory.push_back(comLinearGains);
			jointGainTrajectory.push_back(jointGains);
		}  // end of k loop

	}  // end of i loop


	bag.write("xpp/trajectory_des", startTime, robotStateCartesianTrajectoryMsg);
	bag.close();

//	eigenSwitchingTime.back()(0) = eventTimesPtr->back();

	// ros::init(argc, argv, "switched_state_publisher", ros::init_options::NoSigintHandler);
	// signal(SIGINT, mySigintHandler);
	// ros::NodeHandle n;


	// publish the gaps
//	ros::Publisher gapPublisher = n.advertise<visualization_msgs::Marker>("gap", 100);
//	anymal::publishGapMessages(gapIndicatorPtrs, gapPublisher);



//	ConvertToRosMsg<ocs2_robot_interface_t::STATE_DIM_,ocs2_robot_interface_t::INPUT_DIM_>::ToTrajectoryMsg(
//			timeTrajectoriesStock, stateTrajectoriesStock, 0.02, rosStateTrajectory);
//	std::string trajectoryTopic;
//	ros::param::param<std::string>("trajectory_topic", trajectoryTopic, std::string("/anymalTrajectory"));


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
//		std::ofstream switchingTimeFile(switchingTimeFileName);
//		cereal::XMLOutputArchive switchingTime_xmlArchive(switchingTimeFile);
//		switchingTime_xmlArchive(CEREAL_NVP(eigenSwitchingTime));
//
//		std::ofstream timeTrajectoryFile(timeTrajectoryFileName);
//		cereal::XMLOutputArchive timeTrajectory_xmlArchive(timeTrajectoryFile);
//		timeTrajectory_xmlArchive(CEREAL_NVP(catTimeTrajectory));
//
//		std::ofstream stateTrajectoryFile(stateTrajectoryFileName);
//		cereal::XMLOutputArchive stateTrajectory_xmlArchive(stateTrajectoryFile);
//		stateTrajectory_xmlArchive(CEREAL_NVP(catStateTrajectory));
//
//		std::ofstream inputTrajectoryFile(inputTrajectoryFileName);
//		cereal::XMLOutputArchive inputTrajectory_xmlArchive(inputTrajectoryFile);
//		inputTrajectory_xmlArchive(CEREAL_NVP(catInputTrajectory));
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

}
