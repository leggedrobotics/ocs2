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

#include <ocs2_robotic_examples/examples/quadrotor/QuadrotorInterface.h>
#include <ocs2_robotic_examples/examples/quadrotor/dynamics/QuadrotorSystemDynamics.h>

namespace ocs2 {
namespace quadrotor {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrotorInterface::QuadrotorInterface(const std::string& taskFileFolderName)
{
	taskFile_ = std::string(PACKAGE_PATH) + "/config/quadrotor/" + taskFileFolderName + "/task.info";
	std::cerr << "Loading task file: " << taskFile_ << std::endl;

	libraryFolder_ = std::string(PACKAGE_PATH) + "/auto_generated/quadrotor";
	std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

	// load setting from loading file
	loadSettings(taskFile_);

	// MPC
	setupOptimizer(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrotorInterface::loadSettings(const std::string& taskFile) {

	/*
	 * Default initial condition
	 */
	BASE::loadInitialState(taskFile, initialState_);

	/*
	 * SLQ-MPC settings
	 */
	slqSettings_.loadSettings(taskFile);
	mpcSettings_.loadSettings(taskFile);

	/*
	 * quadrotor parameters
	 */
	QuadrotorParameters<dim_t::scalar_t> quadrotorParameters;
	quadrotorParameters.loadSettings(taskFile);

	/*
	 * Dynamics and derivatives
	 */
	quadrotorSystemDynamicsPtr_.reset(new QuadrotorSystemDynamics(quadrotorParameters));
	quadrotorDynamicsDerivativesPtr_.reset(new QuadrotorDynamicsDerivatives(quadrotorParameters));

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

	quadrotorCostPtr_.reset(new QuadrotorCost(Q_, R_, xNominal_, uNominal_, xFinal_, QFinal_));

	/*
	 * Constraints
	 */
	quadrotorConstraintPtr_.reset(new QuadrotorConstraint);

	/*
	 * Initialization
	 */
	initialInput_ = dim_t::input_vector_t::Zero();
	initialInput_(0) = quadrotorParameters.quadrotorMass_*quadrotorParameters.gravity_;
	quadrotorOperatingPointPtr_.reset( new QuadrotorOperatingPoint(
			BASE::initialState_, BASE::initialInput_) );

	/*
	 * Time partitioning which defines the time horizon and the number of data partitioning
	 */
	scalar_t timeHorizon;
	BASE::definePartitioningTimes(taskFile, timeHorizon,
			numPartitions_, partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrotorInterface::setupOptimizer(const std::string& taskFile) {

	mpcPtr_.reset(new mpc_t(
			quadrotorSystemDynamicsPtr_.get(),
			quadrotorDynamicsDerivativesPtr_.get(),
			quadrotorConstraintPtr_.get(),
			quadrotorCostPtr_.get(),
			quadrotorOperatingPointPtr_.get(),
			partitioningTimes_,
			BASE::slqSettings_,
			BASE::mpcSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrotorInterface::mpc_t::Ptr& QuadrotorInterface::getMPCPtr() {

	return mpcPtr_;
}

} // namespace quadrotor
} // namespace ocs2

