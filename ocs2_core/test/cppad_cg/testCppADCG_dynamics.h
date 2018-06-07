/*
 * testCppADCG_dynamics.h
 *
 *  Created on: May 2, 2018
 *      Author: farbod
 */

#ifndef TESTCPPADCG_DYNAMICS_OCS2_H_
#define TESTCPPADCG_DYNAMICS_OCS2_H_

#include <mutex>
#include <iostream>
#include <functional>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ocs2_core/dynamics/LinearSystemDynamics.h"
#include "LinearSystemDynamicsAD.h"

namespace testCppADCG_dynamics
{

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void checkSystemDynamics(
		const size_t numTests,
		ocs2::DerivativesBase<STATE_DIM, INPUT_DIM> *const linearSystem1,
		ocs2::DerivativesBase<STATE_DIM, INPUT_DIM> *const linearSystem2,
		bool& success) {

	typedef ocs2::DerivativesBase<STATE_DIM, INPUT_DIM> system_dynamics_t;

	typedef typename system_dynamics_t::scalar_t				scalar_t;
	typedef typename system_dynamics_t::state_vector_t			state_vector_t;
	typedef typename system_dynamics_t::state_matrix_t			state_matrix_t;
	typedef typename system_dynamics_t::input_vector_t			input_vector_t;
	typedef typename system_dynamics_t::state_input_matrix_t	state_input_matrix_t;

	success = true;

	const scalar_t precision = 1e-9;

	state_vector_t x;
	input_vector_t u;

	for (size_t it=0; it<numTests; it++) {

		x.setRandom();
		u.setRandom();

		linearSystem1->setCurrentStateAndControl(0.0, x, u);
		linearSystem2->setCurrentStateAndControl(0.0, x, u);

		state_matrix_t A, ad_A;
		linearSystem1->getFlowMapDerivativeState(A);
		linearSystem2->getFlowMapDerivativeState(ad_A);
		if (!A.isApprox(ad_A, precision)) {
			std::cout << "A:    " << A.transpose() << std::endl;
			std::cout << "al_A: " << ad_A.transpose() << std::endl;
			success = false;
		}

		state_input_matrix_t B, ad_B;
		linearSystem1->getFlowMapDerivativeInput(B);
		linearSystem2->getFlowMapDerivativeInput(ad_B);
		if (!B.isApprox(ad_B, precision)) {
			std::cout << "B:    " << B.transpose() << std::endl;
			std::cout << "al_B: " << ad_B.transpose() << std::endl;
			success = false;
		}

		state_matrix_t G, ad_G;
		linearSystem1->getJumpMapDerivativeState(G);
		linearSystem2->getJumpMapDerivativeState(ad_G);
		if (!G.isApprox(ad_G, precision)) {
			std::cout << "G:    " << G.transpose() << std::endl;
			std::cout << "al_G: " << ad_G.transpose() << std::endl;
			success = false;
		}

		state_input_matrix_t H, ad_H;
		linearSystem1->getJumpMapDerivativeInput(H);
		linearSystem2->getJumpMapDerivativeInput(ad_H);
		if (!H.isApprox(ad_H, precision)) {
			std::cout << "H:    " << H.transpose() << std::endl;
			std::cout << "al_H: " << ad_H.transpose() << std::endl;
			success = false;
		}

	} // end of for loop
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
struct dynamics_parameters
{
	enum
	{
		state_dim_ = 4,
		input_dim_ = 2
	};

	typedef ocs2::DerivativesBase<state_dim_, input_dim_> system_dynamics_t;

	typedef typename system_dynamics_t::scalar_t				scalar_t;
	typedef typename system_dynamics_t::state_vector_t			state_vector_t;
	typedef typename system_dynamics_t::state_matrix_t			state_matrix_t;
	typedef typename system_dynamics_t::input_vector_t			input_vector_t;
	typedef typename system_dynamics_t::state_input_matrix_t	state_input_matrix_t;

    // Define cost parameters
	state_matrix_t 			A = state_matrix_t::Random();
	state_input_matrix_t 	B = state_input_matrix_t::Random();
	state_matrix_t 			G = state_matrix_t::Random();;
	state_input_matrix_t 	H = state_input_matrix_t::Random();
};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_dynamics, system_dynamics_test)
{
	enum
	{
		state_dim_ = dynamics_parameters::state_dim_,
		input_dim_ = dynamics_parameters::input_dim_
	};

	typedef ocs2::LinearSystemDynamics<state_dim_, input_dim_>		system_dynamics_t;
	typedef ocs2::LinearSystemDynamicsAD<state_dim_, input_dim_> 	ad_system_dynamics_t;

	dynamics_parameters dynamicsParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	system_dynamics_t linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);

	ad_system_dynamics_t ad_linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);
	ad_linearSystem.createModels("testCppADCG_dynamics", libraryFolder);

	bool success;
	checkSystemDynamics(100, &linearSystem, &ad_linearSystem, success);

	ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_dynamics, clone_test)
{
	enum
	{
		state_dim_ = dynamics_parameters::state_dim_,
		input_dim_ = dynamics_parameters::input_dim_
	};

	typedef ocs2::DerivativesBase<state_dim_, input_dim_> 			base_cost_t;
	typedef ocs2::LinearSystemDynamics<state_dim_, input_dim_> 		system_dynamics_t;
	typedef ocs2::LinearSystemDynamicsAD<state_dim_, input_dim_> 	ad_system_dynamics_t;

	dynamics_parameters dynamicsParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	system_dynamics_t linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);

	ad_system_dynamics_t ad_linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);
	ad_linearSystem.createModels("testCppADCG_dynamics", libraryFolder);

	base_cost_t::Ptr ad_linearSystemPtr(ad_linearSystem.clone());

	bool success;
	checkSystemDynamics(100, &linearSystem, ad_linearSystemPtr.get(), success);

	ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_dynamics, multithread_test)
{
	enum
	{
		state_dim_ = dynamics_parameters::state_dim_,
		input_dim_ = dynamics_parameters::input_dim_
	};

	typedef ocs2::DerivativesBase<state_dim_, input_dim_> 			base_cost_t;
	typedef ocs2::LinearSystemDynamics<state_dim_, input_dim_> 		system_dynamics_t;
	typedef ocs2::LinearSystemDynamicsAD<state_dim_, input_dim_> 	ad_system_dynamics_t;

	dynamics_parameters dynamicsParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	system_dynamics_t linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);

	base_cost_t::Ptr linearSystemPtr(linearSystem.clone());
	base_cost_t::Ptr linearSystemPtr1(linearSystem.clone());

	ad_system_dynamics_t ad_linearSystem(
			dynamicsParam.A, dynamicsParam.B,
			dynamicsParam.G, dynamicsParam.H);
	ad_linearSystem.createModels("testCppADCG_dynamics", libraryFolder);

	base_cost_t::Ptr ad_linearSystemPtr(ad_linearSystem.clone());


	/***************************************************************************
	 *               Multi-thread test
	 **************************************************************************/
	bool success = false;
    std::thread thread1(checkSystemDynamics<state_dim_, input_dim_>, 10000, &linearSystem, &ad_linearSystem, std::ref(success));

    bool successClone = false;
    std::thread thread2(checkSystemDynamics<state_dim_, input_dim_>, 10000, linearSystemPtr.get(), ad_linearSystemPtr.get(), std::ref(successClone));

    if (thread1.joinable())
    	thread1.join();
    if (thread2.joinable())
    	thread2.join();

    ASSERT_TRUE(success && successClone);
}

}  // end of namespace testCppADCG_dynamics

#endif /* TESTCPPADCG_DYNAMICS_OCS2_H_ */
