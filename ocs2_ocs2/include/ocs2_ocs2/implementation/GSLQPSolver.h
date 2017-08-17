/*
 * GSLQPSolver.h
 *
 *  Created on: Jan 18, 2016
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
size_t GSLQPSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::findNearestController(const Eigen::Matrix<double, NUM_Subsystems-1, 1>& enquiry) const  {

	if (parametersBag_.size()==0)  throw  std::runtime_error("controllerStock bag is empty.");

	// evaluating distance
	std::vector<double> distance(parametersBag_.size());
	for (size_t i=0; i<parametersBag_.size(); i++)
		distance[i] = (parametersBag_[i]-enquiry).squaredNorm();

	// min index
	auto it = std::min_element(distance.begin(), distance.end());
	size_t index = std::distance(distance.begin(), it);

	return index;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
void GSLQPSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::run(const double& initTime,
		const state_vector_t& initState, const std::vector<scalar_t>& switchingTimes)  {

	// defining the parameter vector which is the switching times
	Eigen::Matrix<double, NUM_Subsystems-1, 1> parameters = Eigen::VectorXd::Map(switchingTimes.data()+1, NUM_Subsystems-1);

	std::vector<controller_t> controllersStock(NUM_Subsystems);
	if (parametersBag_.size()==0 || options_.warmStartGSLQP_==false) {

		// GLQP initialization
		GLQP_t glqp(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_,
				stateOperatingPoints_, inputOperatingPoints_, systemStockIndex_);

		glqp.run(switchingTimes);

		// GLQP controller
		glqp.getController(controllersStock);
	}
	else {
		// find nearest controller
		size_t index = findNearestController(parameters);
		controllersStock = controllersStockBag_.at(index);
	}

	// GSLQP
	GSLQP_t gslqp(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_,
			controllersStock, systemStockIndex_, options_);
	gslqp.run(initTime, initState, switchingTimes);

	// cost funtion
	gslqp.getValueFuntion(0.0, initState, cost_);

	// cost funtion jacobian
	gslqp.getCostFuntionDerivative(costDerivative_);

	// GSLQP controller
	gslqp.getController(controllersStock_);

	// saving to bag
	parametersBag_.push_back(parameters);
	controllersStockBag_.push_back(controllersStock_);

}

} // namespace ocs2
