#include "ocs2_double_slit_example/DoubleSlitInterface.h"
#include <ros/package.h>

namespace ocs2 {
namespace double_slit {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleSlitInterface::DoubleSlitInterface(const std::string& taskFileFolderName)
{
	taskFile_ = ros::package::getPath("ocs2_double_slit_example") + "/config/" + taskFileFolderName + "/task.info";
	std::cout << "Loading task file: " << taskFile_ << std::endl;

	loadSettings(taskFile_);
	setupOptimizer(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DoubleSlitInterface::loadSettings(const std::string& taskFile) {

	/*
	 * Default initial condition
	 */
	loadInitialState(taskFile, initialState_);

	/*
	 * Dynamics
	 */
	linearSystemDynamicsPtr_.reset(new DoubleSlitDynamics());

	/*
	 * Cost function
	 */
	loadScalar(taskFile, "systemParameters.barrierTimePosition", barrierTimePos_);
	loadScalar(taskFile, "systemParameters.barrierLowerEnd", barrierLowerEnd_);
	loadScalar(taskFile, "systemParameters.barrierUpperEnd", barrierUpperEnd_);
	loadEigenMatrix(taskFile, "Q", Q_);
	loadEigenMatrix(taskFile, "R", R_);
	loadEigenMatrix(taskFile, "Q_final", QFinal_);
	xNominal_ = dim_t::state_vector_t::Zero();
	uNominal_ = dim_t::input_vector_t::Zero();

	std::cerr << "Q:  \n" << Q_ << std::endl;
	std::cerr << "R:  \n" << R_ << std::endl;
	std::cerr << "Q_final:\n" << QFinal_ << std::endl;
	std::cerr << "x_init:   "   << initialState_.transpose() << std::endl;

	auto V = [this](const state_vector_t& x, double t) { return x.dot(this->Q_ * x) + doubleSlitPotentialWall(x,t); };
	auto r = [](const state_vector_t& x, double t) { return input_vector_t::Zero(); };
	auto Phi = [this](const state_vector_t& x) { return x.dot(this->QFinal_ * x); };
	input_vector_t uNominal;
	uNominal.setZero();

	costPtr_.reset(new PathIntegralCostFunction<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>(R_, uNominal, V, r, Phi));

	/*
	 * Constraints
	 */
	linearSystemConstraintPtr_.reset(new DoubleSlitConstraint);

	/*
	 * Initialization
	 */
	linearSystemOperatingPointPtr_.reset(new DoubleSlitOperatingPoint(initialState_, dim_t::input_vector_t::Zero()));

	/*
	 * Time partitioning which defines the time horizon and the number of data partitioning
	 */
	scalar_t timeHorizon;
	definePartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
}

void DoubleSlitInterface::setupOptimizer(const std::string& taskFile) {
  mpcSettings_.loadSettings(taskFile);

  scalar_t rollout_dt;
  loadScalar(taskFile, "systemParameters.rollout_dt", rollout_dt);
  scalar_t gamma;
  loadScalar(taskFile, "pathIntegral.gamma", gamma);
  size_t numSamples;
  loadScalar(taskFile, "pathIntegral.numSamples", numSamples);

  piPtr_.reset(new pi_mpc_t(linearSystemDynamicsPtr_, std::move(costPtr_), *linearSystemConstraintPtr_,  rollout_dt, gamma, numSamples, partitioningTimes_,  mpcSettings_));
}

DoubleSlitInterface::scalar_t DoubleSlitInterface::doubleSlitPotentialWall(dim_t::state_vector_t x, scalar_t t) const {
  if( (std::abs(t - barrierTimePos_) < 0.1) and
      x(0) > barrierLowerEnd_ and
      x(0) < barrierUpperEnd_){
          return std::numeric_limits<scalar_t>::infinity();
        }
  return 0.0;
}

} // namespace double_slit
} // namespace ocs2
