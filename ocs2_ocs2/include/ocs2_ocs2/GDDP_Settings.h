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

#pragma once

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/Integrator.h>

namespace ocs2{

/**
 * This structure contains the settings for the GDDP algorithm.
 */
class GDDP_Settings {

public:
	typedef Dimensions<0,0>::RiccatiIntegratorType RICCATI_INTEGRATOR_TYPE;

	/**
	 * Default constructor.
	 */
	GDDP_Settings()
	: displayInfo_(false)
	, checkNumericalStability_(true)
	, warmStart_(false)
	, useLQForDerivatives_(false)
	, maxNumIterationForLQ_(10)
	, tolGradientDescent_(1e-2)
	, acceptableTolGradientDescent_(1e-1)
	, maxIterationGradientDescent_(20)
	, minLearningRateNLP_(0.05)
	, maxLearningRateNLP_(1.0)
	, minEventTimeDifference_(0.0)
	, nThreads_(4)

	, useNominalTimeForBackwardPass_(false)
	, RiccatiIntegratorType_(RICCATI_INTEGRATOR_TYPE::ODE45)
	, absTolODE_(1e-9)
	, relTolODE_(1e-6)
	, maxNumStepsPerSecond_(5000)
	, minTimeStep_(1e-3)

	{}

	/**
	 * This function loads the "GDDP_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
	 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
	 *
	 * It has the following format:	<br>
	 * slq	<br>
	 * {	<br>
	 *   maxIterationGDDP        value		<br>
	 *   minLearningRateGDDP     value		<br>
	 *   maxLearningRateGDDP     value		<br>
	 *   minRelCostGDDP          value		<br>
	 *   (and so on for the other fields)	<br>
	 * }	<br>
	 *
	 * If a value for a specific field is not defined it will set to the default value defined in "GDDP_Settings".
	 *
	 * @param [in] filename: File name which contains the configuration data.
	 * @param [in] fieldName: Field name which contains the configuration data (the default is slq).
	 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (the default is true).
	 */
	void loadSettings(const std::string& filename, const std::string& fieldName = "gddp", bool verbose = true);

public:
	/****************
	 *** Variables **
	 ****************/

	/** This value determines to display the log output DDP. */
	bool displayInfo_;

	/** Check the numerical stability of the algorithms for debugging purpose. */
	bool checkNumericalStability_;

	/** This value determines to use a warm starting scheme for calculating cost gradients w.r.t. switching times. */
	bool warmStart_;
	/** This value determines to use LQ-based method or sweeping method for calculating cost gradients w.r.t. switching times. */
	bool useLQForDerivatives_;
	/** Maximum number of iterations for LQ-based method  */
	size_t maxNumIterationForLQ_;

	double tolGradientDescent_;
	/** This value determines the termination condition for OCS2 based on the minimum relative changes of the cost.*/
	double acceptableTolGradientDescent_;
	/** This value determines the maximum number of iterations in OCS2 algorithm.*/
	size_t maxIterationGradientDescent_;
	/** This value determines the minimum step size for the line search scheme in OCS2 algorithm.*/
	double minLearningRateNLP_;
	/** This value determines the maximum step size for the line search scheme in OCS2 algorithm.*/
	double maxLearningRateNLP_;
	/** This value determines the minimum accepted difference between to consecutive events times.*/
	double minEventTimeDifference_;

	/** Number of threads used in the multi-threading scheme. */
	size_t nThreads_;

	/** If true, GDDP solves the backward path over the nominal time trajectory. */
	bool useNominalTimeForBackwardPass_;
	/** Riccati integrator type. */
	size_t RiccatiIntegratorType_;
	/** This value determines the absolute tolerance error for ode solvers. */
	double absTolODE_;
	/** This value determines the relative tolerance error for ode solvers. */
	double relTolODE_;
	/** This value determines the maximum number of integration points per a second for ode solvers. */
	size_t maxNumStepsPerSecond_;
	/** The minimum integration time step */
	double minTimeStep_;

}; // end of GDDP_Settings class


inline void GDDP_Settings::loadSettings(const std::string& filename, const std::string& fieldName /*= ilqr*/, bool verbose /*= true*/) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	if(verbose){
		std::cerr << std::endl << " #### GDDP Settings: " << std::endl;
		std::cerr <<" #### =============================================================================" << std::endl;
	}

	try	{
		displayInfo_ = pt.get<bool>(fieldName + ".displayInfo");
		if (verbose) {  std::cerr << " #### Option loader : option 'displayInfo' ......................... " << displayInfo_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'displayInfo' ......................... " << displayInfo_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		checkNumericalStability_ = pt.get<bool>(fieldName + ".checkNumericalStability");
		if (verbose) {  std::cerr << " #### Option loader : option 'checkNumericalStability' ............. " << checkNumericalStability_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'checkNumericalStability' ............. " << checkNumericalStability_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		warmStart_ = pt.get<bool>(fieldName + ".warmStart");
		if (verbose) {  std::cerr << " #### Option loader : option 'warmStart' ........................... " << warmStart_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'warmStart' ........................... " << warmStart_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		useLQForDerivatives_ = pt.get<bool>(fieldName + ".useLQForDerivatives");
		if (verbose) {  std::cerr << " #### Option loader : option 'useLQForDerivatives' ................. " << useLQForDerivatives_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'useLQForDerivatives' ................. " << useLQForDerivatives_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		maxNumIterationForLQ_ = pt.get<size_t>(fieldName + ".maxNumIterationForLQ");
		if (verbose) {  std::cerr << " #### Option loader : option 'maxNumIterationForLQ' ................ " << maxNumIterationForLQ_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'maxNumIterationForLQ' ................ " << maxNumIterationForLQ_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		tolGradientDescent_ = pt.get<double>(fieldName + ".tolGradientDescent");
		if (verbose) {  std::cerr << " #### Option loader : option 'tolGradientDescent' .................. " << tolGradientDescent_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'tolGradientDescent' .................. " << tolGradientDescent_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		acceptableTolGradientDescent_ = pt.get<double>(fieldName + ".acceptableTolGradientDescent");
		if (verbose) {  std::cerr << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << acceptableTolGradientDescent_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'acceptableTolGradientDescent' ........ " << acceptableTolGradientDescent_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		maxIterationGradientDescent_ = pt.get<int>(fieldName + ".maxIterationGradientDescent");
		if (verbose) {  std::cerr << " #### Option loader : option 'maxIterationGradientDescent' ......... " << maxIterationGradientDescent_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'maxIterationGradientDescent' ......... " << maxIterationGradientDescent_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		minLearningRateNLP_ = pt.get<double>(fieldName + ".minLearningRateNLP");
		if (verbose) {  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << minLearningRateNLP_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'minLearningRateNLP' .................. " << minLearningRateNLP_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		maxLearningRateNLP_ = pt.get<double>(fieldName + ".maxLearningRateNLP");
		if (verbose) {  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << maxLearningRateNLP_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'maxLearningRateNLP' .................. " << maxLearningRateNLP_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		minEventTimeDifference_ = pt.get<double>(fieldName + ".minEventTimeDifference");
		if (verbose) {  std::cerr << " #### Option loader : option 'minEventTimeDifference' .............. " << minEventTimeDifference_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'minEventTimeDifference' .............. " << minEventTimeDifference_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		nThreads_ = pt.get<int>(fieldName + ".nThreads");
		if (verbose) {  std::cerr << " #### Option loader : option 'nThreads' ............................ " << nThreads_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'nThreads' ............................ " << nThreads_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		useNominalTimeForBackwardPass_ = pt.get<bool>(fieldName + ".useNominalTimeForBackwardPass");
		if (verbose) {  std::cerr << " #### Option loader : option 'useNominalTimeForBackwardPass' ....... " << useNominalTimeForBackwardPass_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'useNominalTimeForBackwardPass' ....... " << useNominalTimeForBackwardPass_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		RiccatiIntegratorType_ = pt.get<size_t>(fieldName + ".RiccatiIntegratorType");
		if (verbose) {  std::cerr << " #### Option loader : option 'RiccatiIntegratorType' ............... " << RiccatiIntegratorType_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'RiccatiIntegratorType' ............... " << RiccatiIntegratorType_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		absTolODE_ = pt.get<double>(fieldName + ".AbsTolODE");
		if (verbose) {  std::cerr << " #### Option loader : option 'AbsTolODE' ........................... " << absTolODE_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'AbsTolODE' ........................... " << absTolODE_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		relTolODE_ = pt.get<double>(fieldName + ".RelTolODE");
		if (verbose) {  std::cerr << " #### Option loader : option 'RelTolODE' ........................... " << relTolODE_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'RelTolODE' ........................... " << relTolODE_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		maxNumStepsPerSecond_ = pt.get<double>(fieldName + ".maxNumStepsPerSecond");
		if (verbose) {  std::cerr << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << maxNumStepsPerSecond_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'maxNumStepsPerSecond' ................ " << maxNumStepsPerSecond_ << "   \t(default)" << std::endl;
		}
	}

	try	{
		minTimeStep_ = pt.get<double>(fieldName + ".minTimeStep");
		if (verbose) {  std::cerr << " #### Option loader : option 'minTimeStep' ......................... " << minTimeStep_ << std::endl;
		}
	}
	catch (const std::exception& e){
		if (verbose) {  std::cerr << " #### Option loader : option 'minTimeStep' ......................... " << minTimeStep_ << "   \t(default)" << std::endl;
		}
	}

	if(verbose) {
		std::cerr <<" #### =============================================================================" << std::endl;
	}
}

} // namespace ocs2
