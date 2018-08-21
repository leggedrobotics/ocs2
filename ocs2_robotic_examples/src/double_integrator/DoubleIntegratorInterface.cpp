/*
 * DoubleIntegratorInterface.cpp
 *
 *  Created on: May 31, 2018
 *      Author: farbod
 */

#include "ocs2_robotic_examples/examples/double_integrator/DoubleIntegratorInterface.h"

#include <ocs2_core/cost/QuadraticCostFunction.h>

namespace ocs2 {
namespace double_integrator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleIntegratorInterface::DoubleIntegratorInterface(const std::string& taskFileFolderName)
{
	taskFile_ = std::string(PACKAGE_PATH) + "/config/double_integrator/" + taskFileFolderName + "/task.info";
	std::cerr << "Loading task file: " << taskFile_ << std::endl;

	libraryFolder_ = std::string(PACKAGE_PATH) + "/auto_generated/double_integrator";
	std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

	// load setting from loading file
	loadSettings(taskFile_);

	// MPC
	setupOptimizer(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DoubleIntegratorInterface::loadSettings(const std::string& taskFile) {

	/*
	 * Default initial condition
	 */
	loadInitialState(taskFile, initialState_);

	/*
	 * SLQ-MPC settings
	 */
	slqSettings_.loadSettings(taskFile);
	mpcSettings_.loadSettings(taskFile);

	/*
	 * Dynamics
	 */
	dim_t::state_matrix_t A;
	A << 0.0, 1.0,
		 0.0, 0.0;
	dim_t::state_input_matrix_t B;
	B << 0.0, 1.0;
	linearSystemDynamicsPtr_.reset(new DoubleIntegratorDynamics(A, B));
	linearSystemDynamicsDerivativesPtr_.reset(new DoubleIntegratorDynamicsDerivatives(A, B));

	/*
	 * Cost function
	 */
	loadEigenMatrix(taskFile, "Q", Q_);
	loadEigenMatrix(taskFile, "R", R_);
	loadEigenMatrix(taskFile, "Q_final", QFinal_);
	loadEigenMatrix(taskFile, "x_final", xFinal_);
	xNominal_ = dim_t::state_vector_t::Zero();
	uNominal_ = dim_t::input_vector_t::Zero();

	std::cerr << "Q:  \n" << Q_ << std::endl;
	std::cerr << "R:  \n" << R_ << std::endl;
	std::cerr << "Q_final:\n" << QFinal_ << std::endl;
	std::cerr << "x_init:   "   << initialState_.transpose() << std::endl;
	std::cerr << "x_final:  "   << xFinal_.transpose() << std::endl;
	linearSystemCostPtr_.reset(new DoubleIntegratorCost(Q_, R_, QFinal_));

	/*
	 * Constraints
	 */
	linearSystemConstraintPtr_.reset(new DoubleIntegratorConstraint);

	/*
	 * Initialization
	 */
	//	cartPoleOperatingPointPtr_.reset(new CartPoleOperatingPoint(dim_t::state_vector_t::Zero(), dim_t::input_vector_t::Zero()));
	linearSystemOperatingPointPtr_.reset(
			new DoubleIntegratorOperatingPoint(initialState_, dim_t::input_vector_t::Zero()));

	/*
	 * Time partitioning which defines the time horizon and the number of data partitioning
	 */
	scalar_t timeHorizon;
	definePartitioningTimes(taskFile, timeHorizon,
			numPartitions_, partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DoubleIntegratorInterface::setupOptimizer(const std::string& taskFile) {

	mpcPtr_.reset(new mpc_t(
			linearSystemDynamicsPtr_.get(),
			linearSystemDynamicsDerivativesPtr_.get(),
			linearSystemConstraintPtr_.get(),
      linearSystemCostPtr_.get(),
			linearSystemOperatingPointPtr_.get(),
			partitioningTimes_,
			slqSettings_,
			mpcSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleIntegratorInterface::mpc_t::Ptr& DoubleIntegratorInterface::getMPCPtr() {

	return mpcPtr_;
}

} //namespace double_integrator
} // namespace ocs2
