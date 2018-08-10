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

#include <iostream>
#include <mutex>
#include <boost/filesystem.hpp>

#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "../test/cppad_cg/QuadraticCostFunctionAD.h"

template <size_t STATE_DIM, size_t INPUT_DIM>
void checkCostFunction(
		const size_t numTests,
		ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> *const quadraticCost1,
		ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> *const quadraticCost2) {

	typedef ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> 	quadratic_cost_t;

	typedef typename quadratic_cost_t::scalar_t				scalar_t;
	typedef typename quadratic_cost_t::state_vector_t		state_vector_t;
	typedef typename quadratic_cost_t::state_matrix_t		state_matrix_t;
	typedef typename quadratic_cost_t::input_vector_t		input_vector_t;
	typedef typename quadratic_cost_t::input_matrix_t		input_matrix_t;
	typedef typename quadratic_cost_t::input_state_matrix_t	input_state_matrix_t;

	const scalar_t precision = 1e-9;

	state_vector_t x;
	input_vector_t u;

	for (size_t it=0; it<numTests; it++) {

		x.setRandom();
		u.setRandom();

		quadraticCost1->setCurrentStateAndControl(0.0, x, u);
		quadraticCost2->setCurrentStateAndControl(0.0, x, u);

		scalar_t L, ad_L;
		quadraticCost1->evaluate(L);
		quadraticCost2->evaluate(ad_L);
		if (std::abs(L-ad_L) > precision) {
			std::cout << "L:    " << L << std::endl;
			std::cout << "al_L: " << ad_L << std::endl;
		}

		state_vector_t dLdx, ad_dLdx;
		quadraticCost1->stateDerivative(dLdx);
		quadraticCost2->stateDerivative(ad_dLdx);
		if (!dLdx.isApprox(ad_dLdx, precision)) {
			std::cout << "dLdx:    " << dLdx.transpose() << std::endl;
			std::cout << "al_dLdx: " << ad_dLdx.transpose() << std::endl;
		}

		state_matrix_t dLdxx, ad_dLdxx;
		quadraticCost1->stateSecondDerivative(dLdxx);
		quadraticCost2->stateSecondDerivative(ad_dLdxx);
		if (!dLdxx.isApprox(ad_dLdxx, precision)) {
			std::cout << "dLdxx:    \n" << dLdxx << std::endl;
			std::cout << "al_dLdxx: \n" << ad_dLdxx << std::endl;
		}

		input_vector_t dLdu, ad_dLdu;
		quadraticCost1->controlDerivative(dLdu);
		quadraticCost2->controlDerivative(ad_dLdu);
		if (!dLdu.isApprox(ad_dLdu, precision)) {
			std::cout << "dLdu:    " << dLdu.transpose() << std::endl;
			std::cout << "al_dLdu: " << ad_dLdu.transpose() << std::endl;
		}

		input_matrix_t dLduu, ad_dLduu;
		quadraticCost1->controlSecondDerivative(dLduu);
		quadraticCost2->controlSecondDerivative(ad_dLduu);
		if (!dLduu.isApprox(ad_dLduu, precision)) {
			std::cout << "dLduu:    \n" << dLduu << std::endl;
			std::cout << "al_dLduu: \n" << ad_dLduu << std::endl;
		}

		input_state_matrix_t dLdxu, ad_dLdxu;
		quadraticCost1->stateControlDerivative(dLdxu);
		quadraticCost2->stateControlDerivative(ad_dLdxu);
		if (!dLdxu.isApprox(ad_dLdxu, precision)) {
			std::cout << "dLdxu:    \n" << dLdxu << std::endl;
			std::cout << "al_dLdxu: \n" << ad_dLdxu << std::endl;
		}

		scalar_t Phi, ad_Phi;
		quadraticCost1->terminalCost(Phi);
		quadraticCost2->terminalCost(ad_Phi);
		if (std::abs(Phi-ad_Phi) > precision) {
			std::cout << "Phi:    " << Phi << std::endl;
			std::cout << "al_Phi: " << ad_Phi << std::endl;
		}

		state_vector_t dPhidx, ad_dPhidx;
		quadraticCost1->terminalCostStateDerivative(dPhidx);
		quadraticCost2->terminalCostStateDerivative(ad_dPhidx);
		if (!dPhidx.isApprox(ad_dPhidx, precision)) {
			std::cout << "dPhidx:    " << dPhidx.transpose() << std::endl;
			std::cout << "al_dPhidx: " << ad_dPhidx.transpose() << std::endl;
		}

		state_matrix_t dPhidxx, ad_dPhidxx;
		quadraticCost1->terminalCostStateSecondDerivative(dPhidxx);
		quadraticCost2->terminalCostStateSecondDerivative(ad_dPhidxx);
		if (!dPhidxx.isApprox(ad_dPhidxx, precision)) {
			std::cout << "dPhidxx:    \n" << dPhidxx << std::endl;
			std::cout << "al_dPhidxx: \n" << ad_dPhidxx << std::endl;
		}
	} // end of for loop
}

int main(int argc, char** argv)
{

	enum
	{
		state_dim_ = 4,
		input_dim_ = 2
	};

	typedef ocs2::CostFunctionBase<state_dim_, input_dim_> 			base_cost_t;
	typedef ocs2::QuadraticCostFunction<state_dim_, input_dim_> 	quadratic_cost_t;
	typedef ocs2::QuadraticCostFunctionAD<state_dim_, input_dim_> 	ad_quadratic_cost_t;

	typedef base_cost_t::scalar_t			scalar_t;
	typedef base_cost_t::state_vector_t		state_vector_t;
	typedef base_cost_t::state_matrix_t		state_matrix_t;
	typedef base_cost_t::input_vector_t		input_vector_t;
	typedef base_cost_t::input_matrix_t	input_matrix_t;
	typedef base_cost_t::input_state_matrix_t	input_state_matrix_t;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/cppadcg_linux/testCppADCG_cost";

    /***************************************************************************
     *                     Define cost classes
     **************************************************************************/
	state_matrix_t Q = 10.0 * state_matrix_t::Identity();
	input_matrix_t R = input_matrix_t::Identity();;
	state_vector_t x_nominal = state_vector_t::Zero();
	input_vector_t u_nominal = input_vector_t::Zero();
	state_vector_t x_final = state_vector_t::Ones();;
	state_matrix_t Q_final = 4.0 * state_matrix_t::Identity();

	quadratic_cost_t quadraticCost(
			Q, R, x_nominal, u_nominal,
			x_final, Q_final);

	quadratic_cost_t quadraticCost_new = quadraticCost;

	ad_quadratic_cost_t ad_quadraticCost(
			Q, R, x_nominal, u_nominal,
			x_final, Q_final);
	ad_quadraticCost.createModels("testCppADCG_cost", libraryFolder);

	checkCostFunction(100, &quadraticCost, &ad_quadraticCost);

//	std::unique_ptr<ad_quadratic_cost_t> ad_quadraticCostPtr(
//			static_cast<ad_quadratic_cost_t *>(ad_quadraticCost.clone()) );
//	ad_quadraticCostPtr->loadModels("testCppADCG_cost", libraryFolder);
//	checkCostFunction(100, quadraticCost, *ad_quadraticCostPtr);

	base_cost_t::Ptr ad_quadraticCostPtr(ad_quadraticCost.clone());

	checkCostFunction(100, &quadraticCost, ad_quadraticCostPtr.get());

	/***************************************************************************
	 *               Multi-thread test
	 **************************************************************************/

    std::cout << "starting first thread...\n";
    std::thread thread1(checkCostFunction<state_dim_, input_dim_>, 100000, &quadraticCost, &ad_quadraticCost);

    std::cout << "starting second thread...\n";
    std::thread thread2(checkCostFunction<state_dim_, input_dim_>, 100000, &quadraticCost_new, ad_quadraticCostPtr.get());

    std::cout << "waiting for threads to finish..." << std::endl;
    thread1.join();
    thread2.join();

    std::cout << "done!\n";
}

