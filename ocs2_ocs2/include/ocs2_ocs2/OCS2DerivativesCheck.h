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

#ifndef OCS2_DERIVATIVESCHECK_H_
#define OCS2_DERIVATIVESCHECK_H_

#include "ocs2_ocs2/GSLQSolver.h"

namespace ocs2{

/**
 * OCS2 Derivatives Check Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 * @tparam OUTPUT_DIM
 * @tparam NUM_Subsystems
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
class OCS2DerivativesCheck
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef GLQP<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GLQP_t;
	typedef GSLQ<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GSLQ_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM, OUTPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::MP_Options MP_Options_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_t 	  input_state_t;
	typedef typename DIMENSIONS::input_state_array_t input_state_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t 		input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t 		 state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

	typedef Eigen::Matrix<double, NUM_Subsystems-1, 1> parameters_t;

    /**
     * Constructor
     * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
     * @param [in] systemStockIndex
     */
	OCS2DerivativesCheck(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const state_vector_array_t&   stateOperatingPoints,
			const input_vector_array_t& inputOperatingPoints,
			const std::vector<size_t>& systemStockIndex)

	: gslqSolver_(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
	  				stateOperatingPoints, inputOperatingPoints, systemStockIndex, Options_t(), MP_Options_t())
	{}

	virtual ~OCS2DerivativesCheck() {}

    /**
     * Checks!
     * @param [in] initState
     * @param [in] switchingTimes
     */
	void check(const state_vector_t& initState, const std::vector<scalar_t>& switchingTimes)  {
		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

		gslqSolver_.reset();
		gslqSolver_.options().maxIterationGSLQ_ = 30;
		gslqSolver_.options().warmStartGSLQ_ = true;

		gslqSolver_.run(initState, switchingTimes);
		gslqCostDerivative_ = gslqSolver_.costDerivative();
		std::cout << "cost0: " << gslqSolver_.cost() << std::endl;

		// finite difference derivative
		gslqSolver_.options().maxIterationGSLQ_ = 10;

		for (size_t i=0; i<NUM_Subsystems-1; i++) {

			double h = eps_ * std::max(fabs(switchingTimes[i+1]), 1.0);

			std::vector<scalar_t> switchingTimesPlusPerturbed(switchingTimes);
			switchingTimesPlusPerturbed[i+1] += h;
			std::cout << "x+: " << (Eigen::VectorXd::Map(switchingTimesPlusPerturbed.data(),NUM_Subsystems+1)).transpose().format(CleanFmt) << std::endl;
			//
			gslqSolver_.run(initState, switchingTimesPlusPerturbed);
			double costPlusPerturbed = gslqSolver_.cost();
			std::cout << "cost+: " << costPlusPerturbed << std::endl;

			std::vector<scalar_t> switchingTimesMinusPerturbed(switchingTimes);
			switchingTimesMinusPerturbed[i+1] -= h;
			std::cout << "x-: " << (Eigen::VectorXd::Map(switchingTimesMinusPerturbed.data(),NUM_Subsystems+1)).transpose().format(CleanFmt) << std::endl;
			//
			gslqSolver_.run(initState, switchingTimesMinusPerturbed);
			double costMinusPerturbed = gslqSolver_.cost();
			std::cout << "cost+: " << costMinusPerturbed << std::endl;

			fdCostDerivative_(i) = (costPlusPerturbed-costMinusPerturbed) / (2.0*h);
		}

		parameters_t delta = gslqCostDerivative_-fdCostDerivative_;
		Eigen::VectorXd eigenSwitchingTimes = Eigen::VectorXd::Map(switchingTimes.data(),NUM_Subsystems+1);

		std::cout << "Switching time: \t" << eigenSwitchingTimes.transpose().format(CleanFmt) << std::endl;
		std::cout << "GSLQ derivatives: \t" << gslqCostDerivative_.transpose().format(CleanFmt) << std::endl;
		std::cout << "FD derivatives: \t" << fdCostDerivative_.transpose().format(CleanFmt) << std::endl;
		std::cout << "Difference: \t\t" << delta.transpose().format(CleanFmt) << std::endl << std::endl;

	}




private:
	const double eps_= sqrt(1e-2);

	parameters_t gslqCostDerivative_;
	parameters_t fdCostDerivative_;

	GSLQSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> gslqSolver_;

};

} // namespace ocs2

#endif /* OCS2_DERIVATIVESCHECK_H_ */
