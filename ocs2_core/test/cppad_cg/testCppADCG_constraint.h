/*
 * testCppADCG_constraint.h
 *
 *  Created on: May 3, 2018
 *      Author: farbod
 */

#ifndef TESTCPPADCG_CONSTRAINT_OCS2_H_
#define TESTCPPADCG_CONSTRAINT_OCS2_H_

#include <iostream>
#include <mutex>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ocs2_core/constraint/LinearConstraint.h"
#include "LinearConstraintAD.h"

namespace testCppADCG_constraint
{

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool checkSystemDynamics(
		const size_t numTests,
		ocs2::ConstraintBase<STATE_DIM, INPUT_DIM> *const constraint1,
		ocs2::ConstraintBase<STATE_DIM, INPUT_DIM> *const constraint2,
		bool& success) {

	typedef ocs2::ConstraintBase<STATE_DIM, INPUT_DIM> constraint_t;

	typedef typename constraint_t::scalar_t 				scalar_t;
	typedef typename constraint_t::state_vector_t   		state_vector_t;
	typedef typename constraint_t::input_vector_t 			input_vector_t;
	typedef typename constraint_t::state_matrix_t   		state_matrix_t;
	typedef typename constraint_t::state_input_matrix_t 	state_input_matrix_t;
	typedef typename constraint_t::constraint1_vector_t 	constraint1_vector_t;
	typedef typename constraint_t::constraint2_vector_t 	constraint2_vector_t;
	typedef typename constraint_t::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename constraint_t::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename constraint_t::constraint2_state_matrix_t   constraint2_state_matrix_t;

	success = true;

	const scalar_t precision = 1e-9;

	state_vector_t x;
	input_vector_t u;

	for (size_t it=0; it<numTests; it++) {

		x.setRandom();
		u.setRandom();

		constraint1->setCurrentStateAndControl(0.0, x, u);
		constraint2->setCurrentStateAndControl(0.0, x, u);

		size_t n1  = constraint1->numStateInputConstraint(0.0);
		size_t n2  = constraint1->numStateOnlyConstraint(0.0);
		size_t n2f = constraint1->numStateOnlyFinalConstraint(0.0);

		size_t ad_n1  = constraint2->numStateInputConstraint(0.0);
		size_t ad_n2  = constraint2->numStateOnlyConstraint(0.0);
		size_t ad_n2f = constraint2->numStateOnlyFinalConstraint(0.0);

		constraint1_vector_t g1, ad_g1;
		constraint1->getConstraint1(g1);
		constraint2->getConstraint1(ad_g1);
		if (!g1.head(n1).isApprox(ad_g1.head(ad_n1), precision)) {
			std::cout << "g1:    " << g1.head(n1).transpose() << std::endl;
			std::cout << "ad_g1: " << ad_g1.head(ad_n1).transpose() << std::endl;
			success = false;
		}

		constraint1_vector_t g2, ad_g2;
		constraint1->getConstraint2(g2);
		constraint2->getConstraint2(ad_g2);
		if (!g2.head(n2).isApprox(ad_g2.head(ad_n2), precision)) {
			std::cout << "g2:    " << g2.head(n2).transpose() << std::endl;
			std::cout << "ad_g2: " << ad_g2.head(ad_n2).transpose() << std::endl;
			success = false;
		}

		constraint1_vector_t g2f, ad_g2f;
		constraint1->getFinalConstraint2(g2f);
		constraint2->getFinalConstraint2(ad_g2f);
		if (!g2f.head(n2f).isApprox(ad_g2f.head(ad_n2f), precision)) {
			std::cout << "g2f:    " << g2f.head(n2f).transpose() << std::endl;
			std::cout << "ad_g2f: " << ad_g2f.head(ad_n2f).transpose() << std::endl;
			success = false;
		}

		constraint1_state_matrix_t C1, ad_C1;
		constraint1->getConstraint1DerivativesState(C1);
		constraint2->getConstraint1DerivativesState(ad_C1);
		if (!C1.topRows(n1).isApprox(ad_C1.topRows(ad_n1), precision)) {
			std::cout << "C1:    " << C1.topRows(n1).transpose() << std::endl;
			std::cout << "ad_C1: " << ad_C1.topRows(ad_n1).transpose() << std::endl;
			success = false;
		}

		constraint1_input_matrix_t D1, ad_D1;
		constraint1->getConstraint1DerivativesControl(D1);
		constraint2->getConstraint1DerivativesControl(ad_D1);
		if (!D1.topRows(n1).isApprox(ad_D1.topRows(ad_n1), precision)) {
			std::cout << "D1:    " << D1.topRows(n1).transpose() << std::endl;
			std::cout << "ad_D1: " << ad_D1.topRows(ad_n1).transpose() << std::endl;
			success = false;
		}

		constraint2_state_matrix_t F2, ad_F2;
		constraint1->getConstraint2DerivativesState(F2);
		constraint2->getConstraint2DerivativesState(ad_F2);
		if (!F2.topRows(n2).isApprox(ad_F2.topRows(ad_n2), precision)) {
			std::cout << "F2:    " << F2.topRows(n2).transpose() << std::endl;
			std::cout << "ad_F2: " << ad_F2.topRows(ad_n2).transpose() << std::endl;
			success = false;
		}

		constraint2_state_matrix_t F2f, ad_F2f;
		constraint1->getFinalConstraint2DerivativesState(F2f);
		constraint2->getFinalConstraint2DerivativesState(ad_F2f);
		if (!F2f.topRows(n2f).isApprox(ad_F2f.topRows(ad_n2f), precision)) {
			std::cout << "F2f:    " << F2f.topRows(n2f).transpose() << std::endl;
			std::cout << "ad_F2f: " << ad_F2f.topRows(ad_n2f).transpose() << std::endl;
			success = false;
		}

	} // end of for loop
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
struct constraint_parameters
{
	enum
	{
		state_dim_ = 8,
		input_dim_ = 5
	};

	typedef ocs2::ConstraintBase<state_dim_, input_dim_> constraint_t;

	typedef typename constraint_t::scalar_t 				scalar_t;
	typedef typename constraint_t::state_vector_t   		state_vector_t;
	typedef typename constraint_t::input_vector_t 			input_vector_t;
	typedef typename constraint_t::state_matrix_t   		state_matrix_t;
	typedef typename constraint_t::state_input_matrix_t 	state_input_matrix_t;
	typedef typename constraint_t::constraint1_vector_t 	constraint1_vector_t;
	typedef typename constraint_t::constraint2_vector_t 	constraint2_vector_t;
	typedef typename constraint_t::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename constraint_t::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename constraint_t::constraint2_state_matrix_t   constraint2_state_matrix_t;

    // Define constraint parameters
	size_t numStateInputConstraint = input_dim_-1;
	constraint1_vector_t 			e = constraint1_vector_t::Random();
	constraint1_state_matrix_t 		C = constraint1_state_matrix_t::Random();
	constraint1_input_matrix_t 	D = constraint1_input_matrix_t::Random();

	size_t numStateOnlyConstraint = input_dim_-1;
	constraint2_vector_t 		h = constraint2_vector_t::Random();
	constraint2_state_matrix_t 	F = constraint2_state_matrix_t::Random();

	size_t numStateOnlyFinalConstraint = input_dim_-1;
	constraint2_vector_t 		h_f = constraint2_vector_t::Random();
	constraint2_state_matrix_t 	F_f = constraint2_state_matrix_t::Random();

};

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_constraint, constraint_test)
{
	enum
	{
		state_dim_ = constraint_parameters::state_dim_,
		input_dim_ = constraint_parameters::input_dim_
	};

	typedef ocs2::LinearConstraint<state_dim_, input_dim_>		constraint_t;
	typedef ocs2::LinearConstraintAD<state_dim_, input_dim_> 	ad_constraint_t;

	constraint_parameters constraintParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	constraint_t linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);

	ad_constraint_t ad_linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);
	ad_linearConstraint.createModels("testCppADCG_constraint", libraryFolder);

	bool success;
	checkSystemDynamics(100, &linearConstraint, &ad_linearConstraint, success);

	ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_constraint, clone_test)
{
	enum
	{
		state_dim_ = constraint_parameters::state_dim_,
		input_dim_ = constraint_parameters::input_dim_
	};

	typedef ocs2::ConstraintBase<state_dim_, input_dim_> 			base_constraint_t;
	typedef ocs2::LinearConstraint<state_dim_, input_dim_> 		constraint_t;
	typedef ocs2::LinearConstraintAD<state_dim_, input_dim_> 	ad_constraint_t;

	constraint_parameters constraintParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	constraint_t linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);

	ad_constraint_t ad_linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);
	ad_linearConstraint.createModels("testCppADCG_constraint", libraryFolder);

	base_constraint_t::Ptr ad_linearConstraintPtr(ad_linearConstraint.clone());

	bool success;
	checkSystemDynamics(100, &linearConstraint, ad_linearConstraintPtr.get(), success);

	ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST(testCppADCG_constraint, multithread_test)
{
	enum
	{
		state_dim_ = constraint_parameters::state_dim_,
		input_dim_ = constraint_parameters::input_dim_
	};

	typedef ocs2::ConstraintBase<state_dim_, input_dim_> 		base_constraint_t;
	typedef ocs2::LinearConstraint<state_dim_, input_dim_> 		constraint_t;
	typedef ocs2::LinearConstraintAD<state_dim_, input_dim_> 	ad_constraint_t;

	constraint_parameters constraintParam;

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

	constraint_t linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);

	base_constraint_t::Ptr linearConstraintPtr(linearConstraint.clone());
	base_constraint_t::Ptr linearConstraintPtr1(linearConstraint.clone());

	ad_constraint_t ad_linearConstraint(
			constraintParam.numStateInputConstraint,
			constraintParam.e, constraintParam.C, constraintParam.D,
			constraintParam.numStateOnlyConstraint,
			constraintParam.h, constraintParam.F,
			constraintParam.numStateOnlyFinalConstraint,
			constraintParam.h_f, constraintParam.F_f);
	ad_linearConstraint.createModels("testCppADCG_constraint", libraryFolder);

	base_constraint_t::Ptr ad_linearConstraintPtr(ad_linearConstraint.clone());


	/***************************************************************************
	 *               Multi-thread test
	 **************************************************************************/
	bool success = false;
    std::thread thread1(checkSystemDynamics<state_dim_, input_dim_>, 10000, &linearConstraint, &ad_linearConstraint, std::ref(success));

    bool successClone = false;
    std::thread thread2(checkSystemDynamics<state_dim_, input_dim_>, 10000, linearConstraintPtr.get(), ad_linearConstraintPtr.get(), std::ref(successClone));

    if (thread1.joinable())
    	thread1.join();
    if (thread2.joinable())
    	thread2.join();

    ASSERT_TRUE(success && successClone);
}

}  // end of namespace testCppADCG_constraint


#endif /* TESTCPPADCG_CONSTRAINT_OCS2_H_ */
