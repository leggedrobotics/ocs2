/*
 * SLQ_Settings.h
 *
 *  Created on: Feb 2, 2018
 *      Author: farbod
 */

#ifndef SLQ_SETTINGS_OCS2_H_
#define SLQ_SETTINGS_OCS2_H_

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_core/Dimensions.h>

namespace ocs2{

/**
 * This structure contains the settings for the SLQ algorithm.
 */
class SLQ_Settings {

public:
	typedef Dimensions<0,0>::RICCATI_INTEGRATOR_TYPE RICCATI_INTEGRATOR_TYPE;

	/**
	 * Default constructor.
	 */
	SLQ_Settings()
	: maxNumIterationsSLQ_(15)
	, minLearningRateGSLQP_(0.05)
	, maxLearningRateGSLQP_(1.0)
	, lineSearchContractionRate_(0.5)
	, minRelCostGSLQP_(1e-3)
	, stateConstraintPenaltyCoeff_(0.0)
	, stateConstraintPenaltyBase_(1.0)
	, meritFunctionRho_(1.0)
	, constraintStepSize_(1.0)
	, displayInfo_(false)
	, displayShortSummary_(false)
	, warmStartGSLQP_(false)				// GSLQ
	, useLQForDerivatives_(false)			// GSLQ

	, absTolODE_(1e-9)
	, relTolODE_(1e-6)
	, maxNumStepsPerSecond_(5000)
	, minTimeStep_(1e-3)
	, minAbsConstraint1ISE_(1e-3)
	, minRelConstraint1ISE_(1e-3)

	, simulationIsConstrained_(false)
	, noStateConstraints_(false)
	, useMakePSD_(true)

	, displayGradientDescent_(false)		// GSLQ
	, tolGradientDescent_(1e-2)				// GSLQ
	, acceptableTolGradientDescent_(1e-1)	// GSLQ
	, maxIterationGradientDescent_(20)		// GSLQ
	, minLearningRateNLP_(0.05)				// GSLQ
	, maxLearningRateNLP_(1.0)				// GSLQ
	, useAscendingLineSearchNLP_(true)		// GSLQ
	, minEventTimeDifference_(0.0)			// GSLQ

	, useNominalTimeForBackwardPass_(false)
	, RiccatiIntegratorType_(RICCATI_INTEGRATOR_TYPE::ODE45)
	, adams_integrator_dt_(0.001)

	, useMultiThreading_(false)
	, nThreads_(4)
	, debugPrintMP_(false)
	, lsStepsizeGreedy_(true)
	, checkNumericalStability_(true)
	, useRiccatiSolver_(true)
	{}

	/**
	 * This function loads the "SLQ_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
	 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
	 *
	 * It has the following format:	<br>
	 * slq	<br>
	 * {	<br>
	 *   maxIterationGSLQP        value		<br>
	 *   minLearningRateGSLQP     value		<br>
	 *   maxLearningRateGSLQP     value		<br>
	 *   minRelCostGSLQP          value		<br>
	 *   (and so on for the other fields)	<br>
	 * }	<br>
	 *
	 * If a value for a specific field is not defined it will set to the default value defined in "SLQ_Settings".
	 *
	 * @param [in] filename: File name which contains the configuration data.
	 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
	 */
	void loadSettings(const std::string& filename, bool verbose = true);

public:
	/****************
	 *** Variables **
	 ****************/

	/** Maximum number of iterations of SLQ. */
	size_t maxNumIterationsSLQ_;
	/** Minimum number of iterations of SLQ. */
	double minLearningRateGSLQP_;
	/** Maximum learning rate of line-search scheme in SLQ. */
	double maxLearningRateGSLQP_;
	/** Line-search scheme contraction rate. */
	double lineSearchContractionRate_;
	/** This value determines the termination condition based on the minimum relative changes of the cost. */
	double minRelCostGSLQP_;
	/** The penalty function coefficient, \f$\alpha\f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
	double stateConstraintPenaltyCoeff_;
	/** The penalty function base, \f$ a \f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
	double stateConstraintPenaltyBase_;
	/** merit function coefficient. */
	double meritFunctionRho_;
	/** Constant step size for type-1 constraints. */
	double constraintStepSize_;
	/** This value determines to display the log output of SLQ. */
	bool displayInfo_;
	/** This value determines to display the a summary log of SLQ. */
	bool displayShortSummary_;
	/** This value determines to use a warm starting scheme for calculating cost gradients w.r.t. switching times. */
	bool warmStartGSLQP_;
	/** This value determines to use LQ-based method or sweeping method for calculating cost gradients w.r.t. switching times. */
	bool useLQForDerivatives_;

	/** This value determines the absolute tolerance error for ode solvers. */
	double absTolODE_;
	/** This value determines the relative tolerance error for ode solvers. */
	double relTolODE_;
	/** This value determines the maximum number of integration points per a second for ode solvers. */
	size_t maxNumStepsPerSecond_;
	/** The minimum integration time step */
	double minTimeStep_;
	/** This value determines the maximum permitted absolute ISE (Integral of Square Error) for constrained type-1.*/
	double minAbsConstraint1ISE_;
	/** This value determines the maximum permitted relative ISE (Integral of Square Error) for constrained type-1.*/
	double minRelConstraint1ISE_;

	/** Skips calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation.*/
	bool simulationIsConstrained_;

	/** Set true, if a problem does not have state-only constraints. This significantly decreases the runtime of the algorithm. */
	bool noStateConstraints_;

	/** If true SLQ makes sure that PSD matrices remain PSD which increases the numerical stability at the expense of extra computation.*/
	bool useMakePSD_;

	/** This value determines to display the log output of GSLQ. */
	bool displayGradientDescent_;
	double tolGradientDescent_;
	/** This value determines the termination condition for OCS2 based on the minimum relative changes of the cost.*/
	double acceptableTolGradientDescent_;
	/** This value determines the maximum number of iterations in OCS2 algorithm.*/
	size_t maxIterationGradientDescent_;
	/** This value determines the minimum step size for the line search scheme in OCS2 algorithm.*/
	double minLearningRateNLP_;
	/** This value determines the maximum step size for the line search scheme in OCS2 algorithm.*/
	double maxLearningRateNLP_;
	/**
	 * This value determines the the line search scheme to be used in OCS2. \n
	 * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
	 * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
	 * */
	bool useAscendingLineSearchNLP_;
	/** This value determines the minimum accepted difference between to consecutive events times.*/
	double minEventTimeDifference_;

	/** Check the numerical stability of the algorithms for debugging purpose. */
	bool checkNumericalStability_;

	/** If true, SLQ solves the backward path over the nominal time trajectory. */
	bool useNominalTimeForBackwardPass_;
	/** Riccati integrator type. */
	size_t RiccatiIntegratorType_;
	/** Adams integrator's time step. */
	double adams_integrator_dt_;

	/** Use multi threading for the SLQ algorithms. */
	bool useMultiThreading_;
	/** Number of threads used in the multi threading scheme. */
	size_t nThreads_;
	/** Special debugging output for multi threading scheme. */
	bool debugPrintMP_;
	/**
	 * line search options in multi threading scheme.
	 * - True: The largest acceptable step-size will be chosen. This strategy is equivalent to the single core one.
	 * - False: The first acceptable step-size will be chosen.
	 * */
	bool lsStepsizeGreedy_;

	/** If true, SLQ uses ode solver to solve the Riccati equations. Otherwise it uses matrix exponential to solve it. */
	bool useRiccatiSolver_;

}; // end of SLQ_Settings class


inline void SLQ_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	if(verbose){
		std::cerr << std::endl << " #### SLQ Settings: " << std::endl;
		std::cerr <<" #### =============================================================================" << std::endl;
	}

	try	{
		useMultiThreading_ = pt.get<bool>("slq.useMultiThreading");
		if (verbose)  std::cerr << " #### Option loader : option 'useMultiThreading' ................... " << useMultiThreading_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useMultiThreading' ................... " << useMultiThreading_ << "   \t(default)" << std::endl;
	}

	try	{
		nThreads_ = pt.get<int>("slq.nThreads");
		if (verbose)  std::cerr << " #### Option loader : option 'nThreads' ............................ " << nThreads_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'nThreads' ............................ " << nThreads_ << "   \t(default)" << std::endl;
	}

	try	{
		maxNumIterationsSLQ_ = pt.get<int>("slq.maxNumIterationsSLQ");
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumIterationsSLQ' ................. " << maxNumIterationsSLQ_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumIterationsSLQ' ................. " << maxNumIterationsSLQ_ << "   \t(default)" << std::endl;
	}

	try	{
		minLearningRateGSLQP_ = pt.get<double>("slq.minLearningRateGSLQP");
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateGSLQP' ................ " << minLearningRateGSLQP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateGSLQP' ................ " << minLearningRateGSLQP_ << "   \t(default)" << std::endl;
	}

	try	{
		maxLearningRateGSLQP_ = pt.get<double>("slq.maxLearningRateGSLQP");
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateGSLQP' ................ " << maxLearningRateGSLQP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateGSLQP' ................ " << maxLearningRateGSLQP_ << "   \t(default)" << std::endl;
	}

	try	{
		minRelCostGSLQP_ = pt.get<double>("slq.minRelCostGSLQP");
		if (verbose)  std::cerr << " #### Option loader : option 'minRelCostGSLQP' ..................... " << minRelCostGSLQP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minRelCostGSLQP' ..................... " << minRelCostGSLQP_ << "   \t(default)" << std::endl;
	}

	try	{
		stateConstraintPenaltyCoeff_ = pt.get<double>("slq.stateConstraintPenaltyCoeff");
		if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyCoeff' ......... " << stateConstraintPenaltyCoeff_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyCoeff' ......... " << stateConstraintPenaltyCoeff_ << "   \t(default)" << std::endl;
	}

	try	{
		stateConstraintPenaltyBase_ = pt.get<double>("slq.stateConstraintPenaltyBase");
		if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyBase' .......... " << stateConstraintPenaltyBase_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'stateConstraintPenaltyBase' .......... " << stateConstraintPenaltyBase_ << "   \t(default)" << std::endl;
	}

	try	{
		meritFunctionRho_ = pt.get<double>("slq.meritFunctionRho");
		if (verbose)  std::cerr << " #### Option loader : option 'meritFunctionRho' .................... " << meritFunctionRho_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'meritFunctionRho' .................... " << meritFunctionRho_ << "   \t(default)" << std::endl;
	}

	try	{
		constraintStepSize_ = pt.get<double>("slq.constraintStepSize");
		if (verbose)  std::cerr << " #### Option loader : option 'constraintStepSize' .................. " << constraintStepSize_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'constraintStepSize' .................. " << constraintStepSize_ << "   \t(default)" << std::endl;
	}

	try	{
		displayInfo_ = pt.get<bool>("slq.displayInfo");
		if (verbose)  std::cerr << " #### Option loader : option 'displayInfo' ......................... " << displayInfo_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'displayInfo' ......................... " << displayInfo_ << "   \t(default)" << std::endl;
	}

	try	{
		displayShortSummary_ = pt.get<bool>("slq.displayShortSummary");
		if (verbose)  std::cerr << " #### Option loader : option 'displayShortSummary' ................. " << displayShortSummary_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'displayShortSummary' ................. " << displayShortSummary_ << "   \t(default)" << std::endl;
	}

	try	{
		warmStartGSLQP_ = pt.get<bool>("slq.warmStartGSLQP");
		if (verbose)  std::cerr << " #### Option loader : option 'warmStartGSLQP' ...................... " << warmStartGSLQP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'warmStartGSLQP' ...................... " << warmStartGSLQP_ << "   \t(default)" << std::endl;
	}

	try	{
		useLQForDerivatives_ = pt.get<bool>("slq.useLQForDerivatives");
		if (verbose)  std::cerr << " #### Option loader : option 'useLQForDerivatives' ................. " << useLQForDerivatives_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useLQForDerivatives' ................. " << useLQForDerivatives_ << "   \t(default)" << std::endl;
	}

	try	{
		absTolODE_ = pt.get<double>("slq.AbsTolODE");
		if (verbose)  std::cerr << " #### Option loader : option 'AbsTolODE' ........................... " << absTolODE_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'AbsTolODE' ........................... " << absTolODE_ << "   \t(default)" << std::endl;
	}

	try	{
		relTolODE_ = pt.get<double>("slq.RelTolODE");
		if (verbose)  std::cerr << " #### Option loader : option 'RelTolODE' ........................... " << relTolODE_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'RelTolODE' ........................... " << relTolODE_ << "   \t(default)" << std::endl;
	}

	try	{
		maxNumStepsPerSecond_ = pt.get<double>("slq.maxNumStepsPerSecond");
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << maxNumStepsPerSecond_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << maxNumStepsPerSecond_ << "   \t(default)" << std::endl;
	}

	try	{
		simulationIsConstrained_ = pt.get<bool>("slq.simulationIsConstrained");
		if (verbose)  std::cerr << " #### Option loader : option 'simulationIsConstrained' ............. " << simulationIsConstrained_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'simulationIsConstrained' ............. " << simulationIsConstrained_ << "   \t(default)" << std::endl;
	}

	try	{
		noStateConstraints_ = pt.get<bool>("slq.noStateConstraints");
		if (verbose)  std::cerr << " #### Option loader : option 'noStateConstraints' .................. " << noStateConstraints_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'noStateConstraints' .................. " << noStateConstraints_ << "   \t(default)" << std::endl;
	}

	try	{
		useMakePSD_ = pt.get<bool>("slq.useMakePSD");
		if (verbose)  std::cerr << " #### Option loader : option 'useMakePSD' .......................... " << useMakePSD_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useMakePSD' .......................... " << useMakePSD_ << "   \t(default)" << std::endl;
	}

	try	{
		minTimeStep_ = pt.get<double>("slq.minTimeStep");
		if (verbose)  std::cerr << " #### Option loader : option 'minTimeStep' ......................... " << minTimeStep_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minTimeStep' ......................... " << minTimeStep_ << "   \t(default)" << std::endl;
	}

	try	{
		minAbsConstraint1ISE_ = pt.get<double>("slq.minAbsConstraint1ISE");
		if (verbose)  std::cerr << " #### Option loader : option 'minAbsConstraint1ISE' ................ " << minAbsConstraint1ISE_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minAbsConstraint1ISE' ................ " << minAbsConstraint1ISE_ << "   \t(default)" << std::endl;
	}

	try	{
		minRelConstraint1ISE_ = pt.get<double>("slq.minRelConstraint1ISE");
		if (verbose)  std::cerr << " #### Option loader : option 'minRelConstraint1ISE' ................ " << minRelConstraint1ISE_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minRelConstraint1ISE' ................ " << minRelConstraint1ISE_ << "   \t(default)" << std::endl;
	}

	try	{
		displayGradientDescent_ = pt.get<bool>("slq.displayGradientDescent");
		if (verbose)  std::cerr << " #### Option loader : option 'displayGradientDescent' .............. " << displayGradientDescent_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'displayGradientDescent' .............. " << displayGradientDescent_ << "   \t(default)" << std::endl;
	}

	try	{
		tolGradientDescent_ = pt.get<double>("slq.tolGradientDescent");
		if (verbose)  std::cerr << " #### Option loader : option 'tolGradientDescent' .................. " << tolGradientDescent_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'tolGradientDescent' .................. " << tolGradientDescent_ << "   \t(default)" << std::endl;
	}

	try	{
		acceptableTolGradientDescent_ = pt.get<double>("slq.acceptableTolGradientDescent");
		if (verbose)  std::cerr << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << acceptableTolGradientDescent_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << acceptableTolGradientDescent_ << "   \t(default)" << std::endl;
	}

	try	{
		maxIterationGradientDescent_ = pt.get<int>("slq.maxIterationGradientDescent");
		if (verbose)  std::cerr << " #### Option loader : option 'maxIterationGradientDescent' ......... " << maxIterationGradientDescent_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxIterationGradientDescent' ......... " << maxIterationGradientDescent_ << "   \t(default)" << std::endl;
	}

	try	{
		minLearningRateNLP_ = pt.get<double>("slq.minLearningRateNLP");
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << minLearningRateNLP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << minLearningRateNLP_ << "   \t(default)" << std::endl;
	}

	try	{
		maxLearningRateNLP_ = pt.get<double>("slq.maxLearningRateNLP");
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << maxLearningRateNLP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << maxLearningRateNLP_ << "   \t(default)" << std::endl;
	}

	try	{
		useAscendingLineSearchNLP_ = pt.get<bool>("slq.useAscendingLineSearchNLP");
		if (verbose)  std::cerr << " #### Option loader : option 'useAscendingLineSearchNLP' ........... " << useAscendingLineSearchNLP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useAscendingLineSearchNLP' ........... " << useAscendingLineSearchNLP_ << "   \t(default)" << std::endl;
	}

	try	{
		minEventTimeDifference_ = pt.get<double>("slq.minEventTimeDifference");
		if (verbose)  std::cerr << " #### Option loader : option 'minEventTimeDifference' .............. " << minEventTimeDifference_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minEventTimeDifference' .............. " << minEventTimeDifference_ << "   \t(default)" << std::endl;
	}

	try	{
		useNominalTimeForBackwardPass_ = pt.get<bool>("slq.useNominalTimeForBackwardPass");
		if (verbose)  std::cerr << " #### Option loader : option 'useNominalTimeForBackwardPass' ....... " << useNominalTimeForBackwardPass_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useNominalTimeForBackwardPass' ....... " << useNominalTimeForBackwardPass_ << "   \t(default)" << std::endl;
	}

	try	{
		RiccatiIntegratorType_ = pt.get<size_t>("slq.RiccatiIntegratorType");
		if (verbose)  std::cerr << " #### Option loader : option 'RiccatiIntegratorType' ............... " << RiccatiIntegratorType_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'RiccatiIntegratorType' ............... " << RiccatiIntegratorType_ << "   \t(default)" << std::endl;
	}

	try	{
		adams_integrator_dt_ = pt.get<double>("slq.adams_integrator_dt");
		if (verbose)  std::cerr << " #### Option loader : option 'adams_integrator_dt' ................. " << adams_integrator_dt_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'adams_integrator_dt' ................. " << adams_integrator_dt_ << "   \t(default)" << std::endl;
	}

	try	{
		debugPrintMP_ = pt.get<bool>("slq.debugPrintMP");
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintMP' ........................ " << debugPrintMP_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintMP' ........................ " << debugPrintMP_ << "   \t(default)" << std::endl;
	}

	try	{
		lsStepsizeGreedy_ = pt.get<bool>("slq.lsStepsizeGreedy");
		if (verbose)  std::cerr << " #### Option loader : option 'lsStepsizeGreedy' .................... " << lsStepsizeGreedy_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'lsStepsizeGreedy' .................... " << lsStepsizeGreedy_ << "   \t(default)" << std::endl;
	}

	try	{
		checkNumericalStability_ = pt.get<bool>("slq.checkNumericalStability");
		if (verbose)  std::cerr << " #### Option loader : option 'checkNumericalStability' ............. " << checkNumericalStability_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'checkNumericalStability' ............. " << checkNumericalStability_ << "   \t(default)" << std::endl;
	}

	try	{
		useRiccatiSolver_ = pt.get<bool>("slq.useRiccatiSolver");
		if (verbose)  std::cerr << " #### Option loader : option 'useRiccatiSolver' .................... " << useRiccatiSolver_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'useRiccatiSolver' .................... " << useRiccatiSolver_ << "   \t(default)" << std::endl;
	}

	if(verbose)
		std::cerr <<" #### =============================================================================" << std::endl;
}

} // namespace ocs2

#endif /* SLQ_SETTINGS_OCS2_H_ */
