/*
 * MPC_Settings.h
 *
 *  Created on: Jan 31, 2017
 *      Author: farbod
 */

#ifndef MPC_SETTINGS_OCS2_H_
#define MPC_SETTINGS_OCS2_H_

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {

/**
 * This structure holds all setting parameters for the MPC class.
 */
class MPC_Settings {
public:
	/**
	 * Default constructor.
	 */
	MPC_Settings()

	: runtimeNumIterations_(15)
	, initMaxNumIterations_(15)
	, runtimeLearningRate_(1.0)
	, initMaxLearningRate_(1.0)
	, initMinLearningRate_(1.0)
	, debugPrint_(false)
	, coldStart_(false)
	, recedingHorizon_(false)
	, blockwiseMovingHorizon_(false)
	, forwardSimulationTime_(0)   // [ms]
	, useFeedbackPolicy_(false)
	, useParallelRiccatiSolver_(false)
	, rosMsgTimeWindow_(1e+8)
	, adaptiveRosMsgTimeWindow_(false)
	, mpcDesiredFrequency_(-1)  // [Hz]
	, mrtDesiredFrequency_(100)  // [Hz]
	{}

	/**
	 * Loads the MPC settings from a given file.
	 *
	 * @param [in] filename: File name which contains the configuration data.
	 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not
	 * (The default is true).
	 */
	void loadSettings(const std::string& filename, bool verbose = true);

public:
	/****************
	 *** Variables **
	 ****************/
	/** Number of iterations which will be used during MPC regular loop.. */
	size_t runtimeNumIterations_;
	/** Number of iterations which will be used during MPC initial run. */
	size_t initMaxNumIterations_;
	/** Fixed learning rate which will be used during MPC regular loop. */
	double runtimeLearningRate_;
	/** Maximum learning rate which will be used during MPC initial run. */
	double initMaxLearningRate_;
	/** Minimum learning rate which will be used during MPC initial run. */
	double initMinLearningRate_;
	/** This value determines to display the log output of MPC. */
	bool debugPrint_;
	/** This value determines to initialize the SLQ with the controller from previous call
	 * (warm start) or the given operating trajectories (cold start). */
	bool coldStart_;
	/** Either to use the receding horizon MPC or not.*/
	bool recedingHorizon_;
	/** If true the final time of the MPC will increase by the length of a time partition
	 * instead of commonly used scheme where the final time is gradual increased. */
	bool blockwiseMovingHorizon_;
	/** in [ms] */
	int forwardSimulationTime_;
	/** Use either the optimized control policy (true) or the optimized state-input trajectory
	 * (false). */
	bool useFeedbackPolicy_;
	/** If set true, the parallel Riccati solver will be used from the first iteration of SLQ
	 * solver. */
	bool useParallelRiccatiSolver_;
	/** The time window for broadcasting the optimized output (controller and trajectory). */
	double rosMsgTimeWindow_;
	/** Use an adaptive scheme to estimate the time window for broadcasting the optimized output. */
	bool adaptiveRosMsgTimeWindow_;
	/**
	 * MPC loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
	 * If set to a positive number, MPC loop of test will be simulated to run by this
	 * frequency. Note that this might not be the MPC's realtime frequency.
	 */
	double mpcDesiredFrequency_;
	/**
	 * MRT loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
	 * This should always set to a positive number which can be interpreted as the
	 * tracking controller's frequency.
	 */
	double mrtDesiredFrequency_;

}; // end of MPC_Settings class


inline void MPC_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	if(verbose){
		std::cerr << std::endl << " #### MPC Settings: " << std::endl;
		std::cerr <<" #### =============================================================================" << std::endl;
	}

	try	{
		runtimeNumIterations_ = pt.get<size_t>("mpc.runtimeNumIterations");
		if (verbose)  std::cerr << " #### Option loader : option 'runtimeNumIterations' ....... " << runtimeNumIterations_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'runtimeNumIterations' ....... " << runtimeNumIterations_ << " (default)" << std::endl;
	}

	try	{
		initMaxNumIterations_ = pt.get<size_t>("mpc.initMaxNumIterations");
		if (verbose)  std::cerr << " #### Option loader : option 'initMaxNumIterations' ....... " << initMaxNumIterations_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'initMaxNumIterations' ....... " << initMaxNumIterations_ << " (default)" << std::endl;
	}

	try	{
		runtimeLearningRate_ = pt.get<double>("mpc.runtimeLearningRate");
		if (verbose)  std::cerr << " #### Option loader : option 'runtimeLearningRate' ........ " << runtimeLearningRate_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'runtimeLearningRate' ........ " << runtimeLearningRate_ << " (default)" << std::endl;
	}

	try	{
		initMaxLearningRate_ = pt.get<double>("mpc.initMaxLearningRate");
		if (verbose)  std::cerr << " #### Option loader : option 'initMaxLearningRate' ........ " << initMaxLearningRate_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'initMaxLearningRate' ........ " << initMaxLearningRate_ << " (default)" << std::endl;
	}

	try	{
		initMinLearningRate_ = pt.get<double>("mpc.initMinLearningRate");
		if (verbose)  std::cerr << " #### Option loader : option 'initMinLearningRate' ........ " << initMinLearningRate_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'initMinLearningRate' ........ " << initMinLearningRate_ << " (default)" << std::endl;
	}

	try	{
		debugPrint_ = pt.get<bool>("mpc.debugPrint");
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrint' ................. " << debugPrint_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrint' ................. " << debugPrint_ << " (default)" << std::endl;
	}

	try	{
		coldStart_ = pt.get<bool>("mpc.coldStart");
		if (verbose)  std::cerr << " #### Option loader : option 'coldStart' .................. " << coldStart_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'coldStart' .................. " << coldStart_ << " (default)" << std::endl;
	}

	try	{
		recedingHorizon_ = pt.get<bool>("mpc.recedingHorizon");
		if (verbose)  std::cerr << " #### Option loader : option 'recedingHorizon' ............ " << recedingHorizon_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'recedingHorizon' ............ " << recedingHorizon_ << " (default)" << std::endl;
	}

	try	{
		blockwiseMovingHorizon_ = pt.get<bool>("mpc.blockwiseMovingHorizon");
		if (verbose)  std::cerr << " #### Option loader : option 'blockwiseMovingHorizon' ..... " << blockwiseMovingHorizon_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'blockwiseMovingHorizon' ..... " << blockwiseMovingHorizon_ << " (default)" << std::endl;
	}

	try	{
		forwardSimulationTime_ = pt.get<int>("mpc.forwardSimulationTime");
		if (verbose)  std::cerr << " #### Option loader : option 'forwardSimulationTime' ...... " << forwardSimulationTime_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'forwardSimulationTime' ...... " << forwardSimulationTime_ << " (default)" << std::endl;
	}

	try	{
		useFeedbackPolicy_ = pt.get<bool>("mpc.useFeedbackPolicy");
		if (verbose)  std::cerr << " #### Option loader : option 'useFeedbackPolicy' .......... " << useFeedbackPolicy_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useFeedbackPolicy' .......... " << useFeedbackPolicy_ << " (default)" << std::endl;
	}

	try	{
		useParallelRiccatiSolver_ = pt.get<bool>("mpc.useParallelRiccatiSolver");
		if (verbose)  std::cerr << " #### Option loader : option 'useParallelRiccatiSolver' ... " << useParallelRiccatiSolver_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useParallelRiccatiSolver' ... " << useParallelRiccatiSolver_ << " (default)" << std::endl;
	}

	try	{
		rosMsgTimeWindow_ = pt.get<double>("mpc.rosMsgTimeWindow");
		if (verbose)  std::cerr << " #### Option loader : option 'rosMsgTimeWindow' ........... " << rosMsgTimeWindow_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'rosMsgTimeWindow' ........... " << rosMsgTimeWindow_ << " (default)" << std::endl;
	}

	try	{
		adaptiveRosMsgTimeWindow_ = pt.get<bool>("mpc.adaptiveRosMsgTimeWindow");
		if (verbose)  std::cerr << " #### Option loader : option 'adaptiveRosMsgTimeWindow' ... " << adaptiveRosMsgTimeWindow_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'adaptiveRosMsgTimeWindow' ... " << adaptiveRosMsgTimeWindow_ << " (default)" << std::endl;
	}

	try	{
		mpcDesiredFrequency_ = pt.get<double>("mpc.mpcDesiredFrequency");
		if (verbose)  std::cerr << " #### Option loader : option 'mpcDesiredFrequency' ........ " << mpcDesiredFrequency_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'mpcDesiredFrequency' ........ " << mpcDesiredFrequency_ << " (default)" << std::endl;
	}

	try	{
		mrtDesiredFrequency_ = pt.get<double>("mpc.mrtDesiredFrequency");
		if (verbose)  std::cerr << " #### Option loader : option 'mrtDesiredFrequency' ........ " << mrtDesiredFrequency_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'mrtDesiredFrequency' ........ " << mrtDesiredFrequency_ << " (default)" << std::endl;
	}

	if(verbose)
		std::cerr <<" #### =============================================================================" << std::endl;
}

}  // end of ocs2 namespace




#endif /* MPC_SETTINGS_OCS2_H_ */
