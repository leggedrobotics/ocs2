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

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

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
	boost::filesystem::path filePath(__FILE__);
	std::string taskFile = filePath.parent_path().parent_path().generic_string() + "/config/" + std::string(argv[1]) + "/task.info";
	std::cout << "Loading task file " << taskFile << std::endl;

	double slowdown, visTime;
	anymal::OCS2AnymalInterface::loadVisualizationSettings(taskFile, slowdown, visTime);

	// initial state of the switched system
	Eigen::Matrix<double,36,1> initHyQState;
	ocs2::LoadConfigFile::loadMatrix(taskFile, "initialHyQState", initHyQState);

	/******************************************************************************************************/
	anymal::OCS2AnymalInterface anymalOptimization(taskFile);

	auto start = std::chrono::steady_clock::now();

	// run SLQ
	anymalOptimization.runSLQ(0.0, initHyQState);
	// run OCS2
//	anymalOptimization.runOCS2(0.0, initHyQState);
	// run MPC
//	anymalOptimization.runMPC(0.0, initHyQState);

	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;

	std::cout<<"Elapsed time is :  "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()
			<< " ms "<< std::endl;

	double costFunction, constriantISE1, constriantISE2;
	anymalOptimization.getPerformanceIndeces(costFunction, constriantISE1, constriantISE2);

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

	std::vector<switched_model::EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs;
	anymalOptimization.getGapIndicatorPtrs(gapIndicatorPtrs);

	/******************************************************************************************************/
	std::cerr <<"\n======================================" << std::endl;
	std::cerr << "SLQ results: "<< std::endl;
	std::cerr << "The total cost: " << costFunction << std::endl;
	std::cerr << "The type-1 ISE: " << constriantISE1 << std::endl;
	std::cerr << "The type-2 ISE: " << constriantISE2 << std::endl;

	anymal::AnymalKinematics switchedModelKinematics;
	anymal::AnymalCom anymalCom;

	dimension_t::eigen_scalar_array_t eigenInitSwitchingTime(timeTrajectoriesStock.size()+1);
	dimension_t::eigen_scalar_array_t eigenSwitchingTime(timeTrajectoriesStock.size()+1);
	std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > eigenSubsystemsMode(timeTrajectoriesStock.size());
	//
	dimension_t::eigen_scalar_array_t   timeTrajectory;
	dimension_t::state_vector_array_t   stateTrajectory;
	dimension_t::control_vector_array_t inputTrajectory;
	dimension_t::eigen_scalar_array_t   controllerTimeTrajectory;
	typedef Eigen::Matrix<double,1,24*3> pos_com_gain_t;
	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > posGainTrajectory;
	std::vector<pos_com_gain_t, Eigen::aligned_allocator<pos_com_gain_t> > jointGainTrajectory;
	std::vector<Eigen::Matrix<double,12,1>, Eigen::aligned_allocator<Eigen::Matrix<double,12,1>> > feetPositionTrajectory;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > copPositionErrorTrajectory;

	dimension_t::eigen_scalar_array_t iterationCost, iterationISE1, ocs2IterationCost;
	anymalOptimization.getIterationsLog(iterationCost, iterationISE1, ocs2IterationCost);

	// creates a bag file in current directory
	rosbag::Bag bag;
	bag.open("ocs2_anymal_traj.bag", rosbag::bagmode::Write);

	xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg;

	ros::Time::init();
	auto startTime = ros::Time::now();

	robotStateCartesianTrajectoryMsg.header.stamp = startTime;

	for (size_t i=0; i<timeTrajectoriesStock.size(); i++)  {

		if (timeTrajectoriesStock[i].size() == 0)  continue;

		eigenSubsystemsMode[i] << stanceLegSequene[i][0], stanceLegSequene[i][1],
				stanceLegSequene[i][2], stanceLegSequene[i][3];

		eigenInitSwitchingTime[i](0) = initSwitchingTimes[i];
		eigenSwitchingTime[i](0) = switchingTimes[i];

		for (size_t k=0; k<timeTrajectoriesStock[i].size(); k++)  {

			timeTrajectory.push_back( (Eigen::VectorXd(1) << timeTrajectoriesStock[i][k]).finished() );
			stateTrajectory.push_back(stateTrajectoriesStock[i][k]);
			inputTrajectory.push_back(inputTrajectoriesStock[i][k]);

			// calculate the Base pose and local velocities
			anymal::AnymalKinematics::joint_coordinate_t qJoints(stateTrajectoriesStock[i][k].tail<12>());
			anymal::AnymalKinematics::joint_coordinate_t dqJoints(inputTrajectoriesStock[i][k].tail<12>());
			anymal::OCS2AnymalInterface::com_coordinate_t comPose(stateTrajectoriesStock[i][k].head<6>());
			anymal::OCS2AnymalInterface::com_coordinate_t comLocalVelocities(stateTrajectoriesStock[i][k].segment<6>(6));
			anymal::OCS2AnymalInterface::com_coordinate_t basePose;
			anymalCom.calculateBasePose(qJoints, comPose, basePose);
			anymal::OCS2AnymalInterface::com_coordinate_t baseLocalVelocities;
			anymalCom.calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

			// inputTrajectory: contact forces to Origin frame contact forces
			Eigen::Matrix3d o_R_b = switched_model::RotationMatrixOrigintoBase(basePose.head<3>()).transpose();
			for (size_t j=0; j<4; j++) {
				Eigen::Vector3d lambda = inputTrajectoriesStock[i][k].segment<3>(3*j);
				inputTrajectory.back().segment<3>(3*j) = o_R_b * lambda;
			}

			// calculate the feet positions
			anymal::AnymalKinematics::generalized_coordinate_t generalizedCoordinate;
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
//			anymal::SwitchedModelCost anymalCost(stanceLegSequene[i], ...);
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

			point.time_from_start = ros::Duration(timeTrajectoriesStock[i][k]);

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

				point.ee_contact[ee_k] = stanceLegSequene[i][ee_k];
			}

			// const auto stamp = point.time_from_start + startTime;
			const auto stamp = ros::Time(startTime.toSec() + timeTrajectoriesStock[i][k]);

			bag.write("xpp/state_des",stamp, point);
			robotStateCartesianTrajectoryMsg.points.push_back(point);

		}  // end of k loop

		for (size_t k=0; k<controllersStock[i].time_.size(); k++) {
			controllerTimeTrajectory.push_back( (Eigen::VectorXd(1) << controllersStock[i].time_[k]).finished() );

			// CoM equivalent gains
			Eigen::Matrix<double,3,24> ktemp;
			ktemp.setZero();
			for (size_t j=0; j<4; j++)
				if (stanceLegSequene[i][j]==true)
					ktemp += controllersStock[i].k_[k].block<3,24>(3*j,0) / 73.172;

			pos_com_gain_t comLinearGains;
			pos_com_gain_t jointGains;
			for (size_t j=0; j<3; j++) {
				comLinearGains.segment<24>(24*j) = ktemp.row(j);
				jointGains.segment<24>(24*j) = controllersStock[i].k_[k].row(12+j);
			}

			posGainTrajectory.push_back(comLinearGains);
			jointGainTrajectory.push_back(jointGains);
		}  // end of k loop

	}  // end of i loop


	bag.write("xpp/trajectory_des", startTime, robotStateCartesianTrajectoryMsg);
	bag.close();

	eigenInitSwitchingTime.back()(0) = initSwitchingTimes.back();
	eigenSwitchingTime.back()(0) = switchingTimes.back();

	// ros::init(argc, argv, "switched_state_publisher", ros::init_options::NoSigintHandler);
	// signal(SIGINT, mySigintHandler);
	// ros::NodeHandle n;


	// publish the gaps
//	ros::Publisher gapPublisher = n.advertise<visualization_msgs::Marker>("gap", 100);
//	hyq::publishGapMessages(gapIndicatorPtrs, gapPublisher);



//	ConvertToRosMsg<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>::ToTrajectoryMsg(
//			timeTrajectoriesStock, stateTrajectoriesStock, 0.02, rosStateTrajectory);
//	std::string trajectoryTopic;
//	ros::param::param<std::string>("trajectory_topic", trajectoryTopic, std::string("/hyqTrajectory"));


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


}
