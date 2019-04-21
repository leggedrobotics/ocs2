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

#ifndef ILQR_SETTINGS_OCS2_H_
#define ILQR_SETTINGS_OCS2_H_

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_core/Dimensions.h>

#include <ocs2_oc/rollout/Rollout_Settings.h>

namespace ocs2{

/**
 * This structure contains the settings for the ILQR algorithm.
 */
class ILQR_Settings {

public:
	typedef Dimensions<0,0>::RICCATI_INTEGRATOR_TYPE RICCATI_INTEGRATOR_TYPE;

	/**
	 * Default constructor.
	 */
	ILQR_Settings()
	: maxNumIterationsILQR_(15)
	, minLearningRateILQR_(0.05)
	, maxLearningRateILQR_(1.0)
	, lineSearchContractionRate_(0.5)
	, minRelCostILQR_(1e-3)
	, stateConstraintPenaltyCoeff_(0.0)
	, stateConstraintPenaltyBase_(1.0)
	, inequalityConstraintMu_(0.0)
	, inequalityConstraintDelta_(1e-6)
	, meritFunctionRho_(1.0)
	, constraintStepSize_(1.0)
	, displayInfo_(false)
	, displayShortSummary_(false)

	, absTolODE_(1e-9)
	, relTolODE_(1e-6)
	, maxNumStepsPerSecond_(5000)
	, minTimeStep_(1e-3)
	, minAbsConstraint1ISE_(1e-3)
	, minRelConstraint1ISE_(1e-3)

	, simulationIsConstrained_(false)
	, noStateConstraints_(false)
	, useMakePSD_(true)
	, addedRiccatiDiagonal_(1e-5)

	, useMultiThreading_(false)
	, nThreads_(4)
	, threadPriority_(99)
	, debugPrintMT_(false)
	, lsStepsizeGreedy_(true)
	, checkNumericalStability_(true)

	, debugPrintRollout_(false)

	, rolloutSettings_()

	{}

	/**
	 * This function loads the "ILQR_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
	 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
	 *
	 * It has the following format:	<br>
	 * ILQR	<br>
	 * {	<br>
	 *   maxIterationILQR        value		<br>
	 *   minLearningRateILQR     value		<br>
	 *   maxLearningRateILQR     value		<br>
	 *   minRelCostILQR          value		<br>
	 *   (and so on for the other fields)	<br>
	 * }	<br>
	 *
	 * If a value for a specific field is not defined it will set to the default value defined in "ILQR_Settings".
	 *
	 * @param [in] filename: File name which contains the configuration data.
	 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
	 */
	void loadSettings(const std::string& filename, bool verbose = true);

public:
	/****************
	 *** Variables **
	 ****************/

	/** Maximum number of iterations of ILQR. */
	size_t maxNumIterationsILQR_;
	/** Minimum number of iterations of ILQR. */
	double minLearningRateILQR_;
	/** Maximum learning rate of line-search scheme in ILQR. */
	double maxLearningRateILQR_;
	/** Line-search scheme contraction rate. */
	double lineSearchContractionRate_;
	/** This value determines the termination condition based on the minimum relative changes of the cost. */
	double minRelCostILQR_;
	/** The penalty function coefficient, \f$\alpha\f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
	double stateConstraintPenaltyCoeff_;
	/** The penalty function base, \f$ a \f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
	double stateConstraintPenaltyBase_;
	/** Scaling factor, \f$\mu\f$,  for the inequality constraints barrier */
    double inequalityConstraintMu_;
  	/** Threshold parameter, \f$\delta\f$, where the relaxed log barrier function changes from log to quadratic */
    double inequalityConstraintDelta_;
	/** merit function coefficient. */
	double meritFunctionRho_;
	/** Constant step size for type-1 constraints. */
	double constraintStepSize_;
	/** This value determines to display the log output of ILQR. */
	bool displayInfo_;
	/** This value determines to display the a summary log of ILQR. */
	bool displayShortSummary_;

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

	/** If true ILQR makes sure that PSD matrices remain PSD which increases the numerical stability at the expense of extra computation.*/
	bool useMakePSD_;
	/** Add diagonal term to Riccati backward pass for numerical stability. This process is only used when useMakePSD_ set to false.*/
	double addedRiccatiDiagonal_;

	/** Check the numerical stability of the algorithms for debugging purpose. */
	bool checkNumericalStability_;

	/** Use multi threading for the ILQR algorithms. */
	bool useMultiThreading_;
	/** Number of threads used in the multi threading scheme. */
	size_t nThreads_;
	/** Priority of threads used in the multi threading scheme. */
	int threadPriority_;
	/** Special debugging output for multi threading scheme. */
	bool debugPrintMT_;
	/**
	 * line search options in multi threading scheme.
	 * - True: The largest acceptable step-size will be chosen. This strategy is equivalent to the single core one.
	 * - False: The first acceptable step-size will be chosen.
	 * */
	bool lsStepsizeGreedy_;

	/** Printing rollout trajectory for debugging. */
	bool debugPrintRollout_;

	/** This structure contains the settings for forward rollout algorithms. */
	Rollout_Settings rolloutSettings_;

}; // end of ILQR_Settings class


inline void ILQR_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	if(verbose){
		std::cerr << std::endl << " #### ILQR Settings: " << std::endl;
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
		threadPriority_ = pt.get<int>("slq.threadPriority");
		if (verbose)  std::cerr << " #### Option loader : option 'threadPriority' ...................... " << threadPriority_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'threadPriority' ...................... " << threadPriority_ << "   \t(default)" << std::endl;
	}

	try	{
		maxNumIterationsILQR_ = pt.get<int>("slq.maxNumIterationsILQR");
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumIterationsILQR' ................. " << maxNumIterationsILQR_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxNumIterationsILQR' ................. " << maxNumIterationsILQR_ << "   \t(default)" << std::endl;
	}

	try	{
		minLearningRateILQR_ = pt.get<double>("slq.minLearningRateILQR");
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateILQR' ................ " << minLearningRateILQR_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minLearningRateILQR' ................ " << minLearningRateILQR_ << "   \t(default)" << std::endl;
	}

	try	{
		maxLearningRateILQR_ = pt.get<double>("slq.maxLearningRateILQR");
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateILQR' ................ " << maxLearningRateILQR_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'maxLearningRateILQR' ................ " << maxLearningRateILQR_ << "   \t(default)" << std::endl;
	}

	try	{
		minRelCostILQR_ = pt.get<double>("slq.minRelCostILQR");
		if (verbose)  std::cerr << " #### Option loader : option 'minRelCostILQR' ..................... " << minRelCostILQR_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'minRelCostILQR' ..................... " << minRelCostILQR_ << "   \t(default)" << std::endl;
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
		inequalityConstraintMu_ = pt.get<double>("slq.inequalityConstraintMu");
		if (verbose)  std::cerr << " #### Option loader : option 'inequalityConstraintMu' .............. " << inequalityConstraintMu_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'inequalityConstraintMu' .............. " << inequalityConstraintMu_ << "   \t(default)" << std::endl;
	}

	try	{
		inequalityConstraintDelta_ = pt.get<double>("slq.inequalityConstraintDelta");
		if (verbose)  std::cerr << " #### Option loader : option 'inequalityConstraintDelta' ........... " << inequalityConstraintDelta_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'inequalityConstraintDelta' ........... " << inequalityConstraintDelta_ << "   \t(default)" << std::endl;
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
		addedRiccatiDiagonal_ = pt.get<double>("slq.addedRiccatiDiagonal");
		if (verbose)  std::cerr << " #### Option loader : option 'addedRiccatiDiagonal' ................ " << addedRiccatiDiagonal_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'addedRiccatiDiagonal' ................ " << addedRiccatiDiagonal_ << "   \t(default)" << std::endl;
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
		debugPrintMT_ = pt.get<bool>("slq.debugPrintMT");
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintMT' ........................ " << debugPrintMT_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintMT' ........................ " << debugPrintMT_ << "   \t(default)" << std::endl;
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
		debugPrintRollout_ = pt.get<bool>("slq.debugPrintRollout");
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintRollout' ................... " << debugPrintRollout_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### Option loader : option 'debugPrintRollout' ................... " << debugPrintRollout_ << "   \t(default)" << std::endl;
	}

	if(verbose)
		std::cerr <<" #### =============================================================================" << std::endl;


	rolloutSettings_.loadSettings(filename, verbose);
}

} // namespace ocs2

#endif /* ILQR_SETTINGS_OCS2_H_ */
