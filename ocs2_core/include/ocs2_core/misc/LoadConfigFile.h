/*
 * LoadConfigFile.h
 *
 *  Created on: 29.06.2016
 *      Author: farbod
 */

#ifndef OCS2_LOADCONFIGFILE_H_
#define OCS2_LOADCONFIGFILE_H_

#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

class LoadConfigFile
{
public:
	/**
	 * An auxiliary function which loads an Eigen matrix from a file. The file uses property tree data structure with INFO format (refer to www.goo.gl/fV3yWA).
	 *
	 * It has the following format:	<br>
	 * matrixName	<br>
	 * {	<br>
	 *   scaling 1e+0				<br>
	 *   (0,0) value    ; M(0,0)	<br>
	 *   (1,0) value    ; M(1,0)	<br>
	 *   (0,1) value    ; M(0,1)	<br>
	 *   (1,1) value    ; M(1,1)	<br>
	 * } 	<br>
	 *
	 * If a value for a specific element is not defined it will set by default to zero.
	 *
	 * @param [in] filename: File name which contains the configuration data.
	 * @param [in] matrixName: The key name assigned to the matrix in the config file.
	 * @param [out] matrix: The loaded matrix.
	 */
	template <typename Derived>
	static void loadMatrix(const std::string& filename, const std::string& matrixName, Eigen::MatrixBase<Derived>& matrix)
	{
		size_t rows = matrix.rows();
		size_t cols = matrix.cols();

		boost::property_tree::ptree pt;
		boost::property_tree::read_info(filename, pt);

		double scaling = pt.get<double>(matrixName + ".scaling", 1);

		for (size_t i=0; i<rows; i++)
		{
			for (size_t j=0; j<cols; j++)
			{
				matrix(i,j) = scaling*pt.get<double>(matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")" , 0.0);
			}
		}
	}

//	/**
//	 * This function loads the "Options" structure from a config file. This file contains the settings for the SQL and OCS2 algorithms.
//	 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
//	 *
//	 * It has the following format:	<br>
//	 * ocs2	<br>
//	 * {	<br>
//	 *   maxIterationGSLQP        value		<br>
//	 *   minLearningRateGSLQP     value		<br>
//	 *   maxLearningRateGSLQP     value		<br>
//	 *   minRelCostGSLQP          value		<br>
//	 *   (and so on for the other fields)	<br>
//	 * }	<br>
//	 *
//	 * If a value for a specific field is not defined it will set to the default value defined in "Dimensions::Options".
//	 *
//	 * @param [in] filename: File name which contains the configuration data.
//	 * @param [out] opt: The settings for the SQL and OCS2 algorithms.
//	 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
//	 */
//	template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM=STATE_DIM>
//	static void loadOptions(const std::string& filename, typename Dimensions<STATE_DIM, INPUT_DIM, OUTPUT_DIM>::Options& opt, bool verbose = true)
//	{
//		boost::property_tree::ptree pt;
//		boost::property_tree::read_info(filename, pt);
//
//		if(verbose){
//			std::cerr <<" #### OCS2 Options: " << std::endl;
//			std::cerr <<" #### ==============================================================================================" << std::endl;
//		}
//
//		try	{
//			opt.nThreads_ = pt.get<int>("ocs2.nThreads");
//			if (verbose)  std::cout << " #### Option loader : option 'nThreads' ............................ " << opt.nThreads_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'nThreads' ............................ " << opt.nThreads_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.maxIterationGSLQP_ = pt.get<int>("ocs2.maxIterationGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'maxIterationGSLQP' ................... " << opt.maxIterationGSLQP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'maxIterationGSLQP' ................... " << opt.maxIterationGSLQP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minLearningRateGSLQP_ = pt.get<double>("ocs2.minLearningRateGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'minLearningRateGSLQP' ................ " << opt.minLearningRateGSLQP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minLearningRateGSLQP' ................ " << opt.minLearningRateGSLQP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.maxLearningRateGSLQP_ = pt.get<double>("ocs2.maxLearningRateGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'maxLearningRateGSLQP' ................ " << opt.maxLearningRateGSLQP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'maxLearningRateGSLQP' ................ " << opt.maxLearningRateGSLQP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minRelCostGSLQP_ = pt.get<double>("ocs2.minRelCostGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'minRelCostGSLQP' ..................... " << opt.minRelCostGSLQP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minRelCostGSLQP' ..................... " << opt.minRelCostGSLQP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.stateConstraintPenaltyCoeff_ = pt.get<double>("ocs2.stateConstraintPenaltyCoeff");
//			if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyCoeff' ......... " << opt.stateConstraintPenaltyCoeff_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyCoeff' ......... " << opt.stateConstraintPenaltyCoeff_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.stateConstraintPenaltyBase_ = pt.get<double>("ocs2.stateConstraintPenaltyBase");
//			if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyBase' .......... " << opt.stateConstraintPenaltyBase_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyBase' .......... " << opt.stateConstraintPenaltyBase_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.meritFunctionRho_ = pt.get<double>("ocs2.meritFunctionRho");
//			if (verbose)  std::cout << " #### Option loader : option 'meritFunctionRho' .................... " << opt.meritFunctionRho_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'meritFunctionRho' .................... " << opt.meritFunctionRho_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.constraintStepSize_ = pt.get<double>("ocs2.constraintStepSize");
//			if (verbose)  std::cout << " #### Option loader : option 'constraintStepSize' .................. " << opt.constraintStepSize_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'constraintStepSize' .................. " << opt.constraintStepSize_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.dispayInfo_ = pt.get<bool>("ocs2.dispayGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'dispayInfo' ......................... " << opt.dispayInfo_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'dispayInfo' ......................... " << opt.dispayInfo_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.displayShortSummary_ = pt.get<bool>("ocs2.displayShortSummary");
//			if (verbose)  std::cout << " #### Option loader : option 'displayShortSummary' ................. " << opt.displayShortSummary_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'displayShortSummary' ................. " << opt.displayShortSummary_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.warmStartGSLQP_ = pt.get<bool>("ocs2.warmStartGSLQP");
//			if (verbose)  std::cout << " #### Option loader : option 'warmStartGSLQP' ...................... " << opt.warmStartGSLQP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'warmStartGSLQP' ...................... " << opt.warmStartGSLQP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.useLQForDerivatives_ = pt.get<bool>("ocs2.useLQForDerivatives");
//			if (verbose)  std::cout << " #### Option loader : option 'useLQForDerivatives' ................. " << opt.useLQForDerivatives_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'useLQForDerivatives' ................. " << opt.useLQForDerivatives_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.AbsTolODE_ = pt.get<double>("ocs2.AbsTolODE");
//			if (verbose)  std::cout << " #### Option loader : option 'AbsTolODE' ........................... " << opt.AbsTolODE_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'AbsTolODE' ........................... " << opt.AbsTolODE_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.RelTolODE_ = pt.get<double>("ocs2.RelTolODE");
//			if (verbose)  std::cout << " #### Option loader : option 'RelTolODE' ........................... " << opt.RelTolODE_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'RelTolODE' ........................... " << opt.RelTolODE_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.maxNumStepsPerSecond_ = pt.get<size_t>("ocs2.maxNumStepsPerSecond");
//			if (verbose)  std::cout << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << opt.maxNumStepsPerSecond_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << opt.maxNumStepsPerSecond_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.simulationIsConstrained_ = pt.get<bool>("ocs2.simulationIsConstrained");
//			if (verbose)  std::cout << " #### Option loader : option 'simulationIsConstrained' ............. " << opt.simulationIsConstrained_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'simulationIsConstrained' ............. " << opt.simulationIsConstrained_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minSimulationTimeDuration_ = pt.get<double>("ocs2.minSimulationTimeDuration");
//			if (verbose)  std::cout << " #### Option loader : option 'minSimulationTimeDuration' ........... " << opt.minSimulationTimeDuration_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minSimulationTimeDuration' ........... " << opt.minSimulationTimeDuration_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minAbsConstraint1ISE_ = pt.get<double>("ocs2.minAbsConstraint1ISE");
//			if (verbose)  std::cout << " #### Option loader : option 'minAbsConstraint1ISE' ................ " << opt.minAbsConstraint1ISE_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minAbsConstraint1ISE' ................ " << opt.minAbsConstraint1ISE_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minRelConstraint1ISE_ = pt.get<double>("ocs2.minRelConstraint1ISE");
//			if (verbose)  std::cout << " #### Option loader : option 'minRelConstraint1ISE' ................ " << opt.minRelConstraint1ISE_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minRelConstraint1ISE' ................ " << opt.minRelConstraint1ISE_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.displayGradientDescent_ = pt.get<bool>("ocs2.displayGradientDescent");
//			if (verbose)  std::cout << " #### Option loader : option 'displayGradientDescent' .............. " << opt.displayGradientDescent_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'displayGradientDescent' .............. " << opt.displayGradientDescent_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.tolGradientDescent_ = pt.get<double>("ocs2.tolGradientDescent");
//			if (verbose)  std::cout << " #### Option loader : option 'tolGradientDescent' .................. " << opt.tolGradientDescent_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'tolGradientDescent' .................. " << opt.tolGradientDescent_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.acceptableTolGradientDescent_ = pt.get<double>("ocs2.acceptableTolGradientDescent");
//			if (verbose)  std::cout << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << opt.acceptableTolGradientDescent_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << opt.acceptableTolGradientDescent_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.maxIterationGradientDescent_ = pt.get<int>("ocs2.maxIterationGradientDescent");
//			if (verbose)  std::cout << " #### Option loader : option 'maxIterationGradientDescent' ......... " << opt.maxIterationGradientDescent_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'maxIterationGradientDescent' ......... " << opt.maxIterationGradientDescent_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.minLearningRateNLP_ = pt.get<double>("ocs2.minLearningRateNLP");
//			if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << opt.minLearningRateNLP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << opt.minLearningRateNLP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.maxLearningRateNLP_ = pt.get<double>("ocs2.maxLearningRateNLP");
//			if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << opt.maxLearningRateNLP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << opt.maxLearningRateNLP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.useAscendingLineSearchNLP_ = pt.get<bool>("ocs2.useAscendingLineSearchNLP");
//			if (verbose)  std::cerr << " #### Option loader : option 'useAscendingLineSearchNLP' ........... " << opt.useAscendingLineSearchNLP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cerr << " #### Option loader : option 'useAscendingLineSearchNLP' ........... " << opt.useAscendingLineSearchNLP_ << " (default)" << std::endl;
//		}
//
//		try	{
//			opt.minAcceptedSwitchingTimeDifference_ = pt.get<double>("ocs2.minAcceptedSwitchingTimeDifference");
//			if (verbose)  std::cout << " #### Option loader : option 'minAcceptedSwitchingTimeDifference' .. " << opt.minAcceptedSwitchingTimeDifference_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'minAcceptedSwitchingTimeDifference' .. " << opt.minAcceptedSwitchingTimeDifference_ << " (default)" << std::endl;
//		}
//
//		try	{
//			opt.RiccatiIntegratorType_ = pt.get<size_t>("ocs2.RiccatiIntegratorType");
//			if (verbose)  std::cout << " #### Option loader : option 'RiccatiIntegratorType' ............... " << opt.RiccatiIntegratorType_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'RiccatiIntegratorType' ............... " << opt.RiccatiIntegratorType_ << " (default)" << std::endl;
//		}
//
//		try	{
//			opt.adams_integrator_dt_ = pt.get<double>("ocs2.adams_integrator_dt");
//			if (verbose)  std::cout << " #### Option loader : option 'adams_integrator_dt' ................. " << opt.adams_integrator_dt_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'adams_integrator_dt' ................. " << opt.adams_integrator_dt_ << " (default)" << std::endl;
//		}
//
//		try	{
//			opt.debugPrintMP_ = pt.get<bool>("ocs2.debugPrintMP");
//			if (verbose)  std::cout << " #### Option loader : option 'debugPrintMP' ........................ " << opt.debugPrintMP_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'debugPrintMP' ........................ " << opt.debugPrintMP_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.lsStepsizeGreedy_ = pt.get<bool>("ocs2.lsStepsizeGreedy");
//			if (verbose)  std::cout << " #### Option loader : option 'lsStepsizeGreedy' .................... " << opt.lsStepsizeGreedy_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'lsStepsizeGreedy' .................... " << opt.lsStepsizeGreedy_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.checkNumericalStability_ = pt.get<bool>("ocs2.checkNumericalStability");
//			if (verbose)  std::cout << " #### Option loader : option 'checkNumericalStability' ............. " << opt.checkNumericalStability_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'checkNumericalStability' ............. " << opt.checkNumericalStability_ << "\t(default)" << std::endl;
//		}
//
//		try	{
//			opt.useRiccatiSolver_ = pt.get<bool>("ocs2.useRiccatiSolver");
//			if (verbose)  std::cout << " #### Option loader : option 'useRiccatiSolver' .................... " << opt.useRiccatiSolver_ << std::endl;
//		}
//		catch (const std::exception& e){
//			if (verbose)  std::cout << " #### Option loader : option 'useRiccatiSolver' .................... " << opt.useRiccatiSolver_ << "\t(default)" << std::endl;
//		}
//
//		if(verbose)
//			std::cerr <<" #### ========================================================================================== ####" << std::endl;
//	}

};

}  // end of ocs2 namespace

#endif /* OCS2_LOADCONFIGFILE_H_ */
