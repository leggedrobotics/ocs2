/*
 * testCppADCG_cost.h
 *
 *  Created on: Apr 27, 2018
 *      Author: farbod
 */

#ifndef TESTCPPADCG_COST_OCS2_H_
#define TESTCPPADCG_COST_OCS2_H_

#include <iostream>
#include <mutex>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "QuadraticCostFunctionAD.h"

namespace testCppADCG_cost
{

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool checkCostFunction(
		const size_t numTests,
		ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> *const quadraticCost1,
		ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> *const quadraticCost2) {

	typedef ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> 	quadratic_cost_t;

	typedef typename quadratic_cost_t::scalar_t				scalar_t;
	typedef typename quadratic_cost_t::state_vector_t		state_vector_t;
	typedef typename quadratic_cost_t::state_matrix_t		state_matrix_t;
	typedef typename quadratic_cost_t::input_vector_t		input_vector_t;
	typedef typename quadratic_cost_t::control_matrix_t		control_matrix_t;
	typedef typename quadratic_cost_t::control_feedback_t	control_feedback_t;

	bool success = true;

	const scalar_t precision = 1e-9;

	state_vector_t x;
	input_vector_t u;

	for (size_t it=0; it<numTests; it++) {

		x.setRandom();
		u.setRandom();

		quadraticCost1->setCurrentStateAndControl(0.0, x, u);
		quadraticCost2->setCurrentStateAndControl(0.0, x, u);

		scalar_t L, ad_L;
		quadraticCost1->getIntermediateCost(L);
		quadraticCost2->getIntermediateCost(ad_L);
		if (std::abs(L-ad_L) > precision) {
			std::cout << "L:    " << L << std::endl;
			std::cout << "al_L: " << ad_L << std::endl;
			success = false;
		}

		state_vector_t dLdx, ad_dLdx;
		quadraticCost1->getIntermediateCostDerivativeState(dLdx);
		quadraticCost2->getIntermediateCostDerivativeState(ad_dLdx);
		if (!dLdx.isApprox(ad_dLdx, precision)) {
			std::cout << "dLdx:    " << dLdx.transpose() << std::endl;
			std::cout << "al_dLdx: " << ad_dLdx.transpose() << std::endl;
			success = false;
		}

		state_matrix_t dLdxx, ad_dLdxx;
		quadraticCost1->getIntermediateCostSecondDerivativeState(dLdxx);
		quadraticCost2->getIntermediateCostSecondDerivativeState(ad_dLdxx);
		if (!dLdxx.isApprox(ad_dLdxx, precision)) {
			std::cout << "dLdxx:    \n" << dLdxx << std::endl;
			std::cout << "al_dLdxx: \n" << ad_dLdxx << std::endl;
			success = false;
		}

		input_vector_t dLdu, ad_dLdu;
		quadraticCost1->getIntermediateCostDerivativeInput(dLdu);
		quadraticCost2->getIntermediateCostDerivativeInput(ad_dLdu);
		if (!dLdu.isApprox(ad_dLdu, precision)) {
			std::cout << "dLdu:    " << dLdu.transpose() << std::endl;
			std::cout << "al_dLdu: " << ad_dLdu.transpose() << std::endl;
			success = false;
		}

		control_matrix_t dLduu, ad_dLduu;
		quadraticCost1->getIntermediateCostSecondDerivativeInput(dLduu);
		quadraticCost2->getIntermediateCostSecondDerivativeInput(ad_dLduu);
		if (!dLduu.isApprox(ad_dLduu, precision)) {
			std::cout << "dLduu:    \n" << dLduu << std::endl;
			std::cout << "al_dLduu: \n" << ad_dLduu << std::endl;
			success = false;
		}

		control_feedback_t dLdxu, ad_dLdxu;
		quadraticCost1->getIntermediateCostDerivativeInputState(dLdxu);
		quadraticCost2->getIntermediateCostDerivativeInputState(ad_dLdxu);
		if (!dLdxu.isApprox(ad_dLdxu, precision)) {
			std::cout << "dLdxu:    \n" << dLdxu << std::endl;
			std::cout << "al_dLdxu: \n" << ad_dLdxu << std::endl;
			success = false;
		}

		scalar_t Phi, ad_Phi;
		quadraticCost1->getTerminalCost(Phi);
		quadraticCost2->getTerminalCost(ad_Phi);
		if (std::abs(Phi-ad_Phi) > precision) {
			std::cout << "Phi:    " << Phi << std::endl;
			std::cout << "al_Phi: " << ad_Phi << std::endl;
			success = false;
		}

		state_vector_t dPhidx, ad_dPhidx;
		quadraticCost1->getTerminalCostDerivativeState(dPhidx);
		quadraticCost2->getTerminalCostDerivativeState(ad_dPhidx);
		if (!dPhidx.isApprox(ad_dPhidx, precision)) {
			std::cout << "dPhidx:    " << dPhidx.transpose() << std::endl;
			std::cout << "al_dPhidx: " << ad_dPhidx.transpose() << std::endl;
			success = false;
		}

		state_matrix_t dPhidxx, ad_dPhidxx;
		quadraticCost1->getTerminalCostSecondDerivativeState(dPhidxx);
		quadraticCost2->getTerminalCostSecondDerivativeState(ad_dPhidxx);
		if (!dPhidxx.isApprox(ad_dPhidxx, precision)) {
			std::cout << "dPhidxx:    \n" << dPhidxx << std::endl;
			std::cout << "al_dPhidxx: \n" << ad_dPhidxx << std::endl;
			success = false;
		}
	} // end of for loop

	return success;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
struct cost_parameters
{
	enum
	{
		state_dim_ = 4,
		input_dim_ = 2
	};

	typedef ocs2::CostFunctionBase<state_dim_, input_dim_> base_cost_t;

	typedef base_cost_t::scalar_t			scalar_t;
	typedef base_cost_t::state_vector_t		state_vector_t;
	typedef base_cost_t::state_matrix_t		state_matrix_t;
	typedef base_cost_t::input_vector_t		input_vector_t;
	typedef base_cost_t::control_matrix_t	control_matrix_t;
	typedef base_cost_t::control_feedback_t	control_feedback_t;

    // Define cost parameters
	state_matrix_t Q = 10.0 * state_matrix_t::Identity();
	control_matrix_t R = control_matrix_t::Identity();;
	state_vector_t x_nominal = state_vector_t::Zero();
	input_vector_t u_nominal = input_vector_t::Zero();
	state_vector_t x_final = state_vector_t::Ones();;
	state_matrix_t Q_final = 4.0 * state_matrix_t::Identity();
};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
//TEST(testCppADCG_cost, quadratic_cost_test)
//{
//	enum
//	{
//		state_dim_ = cost_parameters::state_dim_,
//		input_dim_ = cost_parameters::input_dim_
//	};
//
//	typedef ocs2::CostFunctionBase<state_dim_, input_dim_> 			base_cost_t;
//	typedef ocs2::QuadraticCostFunction<state_dim_, input_dim_> 	quadratic_cost_t;
//	typedef ocs2::QuadraticCostFunctionAD<state_dim_, input_dim_> 	ad_quadratic_cost_t;
//
//	cost_parameters costParam;
//
//    /***************************************************************************
//     *               path to save the generated dynamic-library
//     **************************************************************************/
//    boost::filesystem::path filePath(__FILE__);
//    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
//
//	quadratic_cost_t quadraticCost(
//			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
//			costParam.x_final, costParam.Q_final);
//
//	ad_quadratic_cost_t ad_quadraticCost(
//			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
//			costParam.x_final, costParam.Q_final);
//	ad_quadraticCost.createModels("testCppADCG_cost", libraryFolder);
//
//	bool success = checkCostFunction(100, &quadraticCost, &ad_quadraticCost);
//
//	ASSERT_TRUE(success);
//}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
//TEST(testCppADCG_cost, clone_test)
//{
//	enum
//	{
//		state_dim_ = cost_parameters::state_dim_,
//		input_dim_ = cost_parameters::input_dim_
//	};
//
//	typedef ocs2::CostFunctionBase<state_dim_, input_dim_> 			base_cost_t;
//	typedef ocs2::QuadraticCostFunction<state_dim_, input_dim_> 	quadratic_cost_t;
//	typedef ocs2::QuadraticCostFunctionAD<state_dim_, input_dim_> 	ad_quadratic_cost_t;
//
//	cost_parameters costParam;
//
//    /***************************************************************************
//     *               path to save the generated dynamic-library
//     **************************************************************************/
//    boost::filesystem::path filePath(__FILE__);
//    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
//
//	quadratic_cost_t quadraticCost(
//			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
//			costParam.x_final, costParam.Q_final);
//
//	ad_quadratic_cost_t ad_quadraticCost(
//			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
//			costParam.x_final, costParam.Q_final);
//	ad_quadraticCost.createModels("testCppADCG_cost", libraryFolder);
//
//	base_cost_t::Ptr ad_quadraticCostPtr(ad_quadraticCost.clone());
//
//	bool success = checkCostFunction(100, &quadraticCost, ad_quadraticCostPtr.get());
//
//	ASSERT_TRUE(success);
//}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_cost, multithread_test)
{
	enum
	{
		state_dim_ = cost_parameters::state_dim_,
		input_dim_ = cost_parameters::input_dim_
	};

	typedef ocs2::CostFunctionBase<state_dim_, input_dim_> 			base_cost_t;
	typedef ocs2::QuadraticCostFunction<state_dim_, input_dim_> 	quadratic_cost_t;
	typedef ocs2::QuadraticCostFunctionAD<state_dim_, input_dim_> 	ad_quadratic_cost_t;

	cost_parameters costParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	quadratic_cost_t quadraticCost(
			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
			costParam.x_final, costParam.Q_final);

	base_cost_t::Ptr quadraticCostPtr(quadraticCost.clone());
	base_cost_t::Ptr quadraticCostPtr1(quadraticCost.clone());

	ad_quadratic_cost_t ad_quadraticCost(
			costParam.Q, costParam.R, costParam.x_nominal, costParam.u_nominal,
			costParam.x_final, costParam.Q_final);

	ad_quadraticCost.createModels("testCppADCG_cost", libraryFolder);

	base_cost_t::Ptr ad_quadraticCostPtr(ad_quadraticCost.clone());


	/***************************************************************************
	 *               Multi-thread test
	 **************************************************************************/
    std::thread thread1(checkCostFunction<state_dim_, input_dim_>, 10000, &quadraticCost, &ad_quadraticCost);

    std::thread thread2(checkCostFunction<state_dim_, input_dim_>, 10000, quadraticCostPtr.get(), ad_quadraticCostPtr.get());

    if (thread1.joinable())
    	thread1.join();
    if (thread2.joinable())
    	thread2.join();
}

}  // end of namespace testCppADCG_cost

#endif /* TESTCPPADCG_COST_OCS2_H_ */
