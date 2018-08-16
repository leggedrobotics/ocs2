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

#ifndef TESTCPPADCG_SIMPLE_LINUX_OCS2_H_
#define TESTCPPADCG_SIMPLE_LINUX_OCS2_H_

#include <iostream>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>

namespace cppadcg_simple_test {

//enum DIM : size_t {
//	DOMAIN_DIM_ = 3,
//	RANGE_DIM_  = 2
//};

const size_t DOMAIN_DIM_ = 3;
const size_t RANGE_DIM_  = 2;


template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> func(const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& x) {

	Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> y(RANGE_DIM_);

	y(0) = ( x(0) + x(1) * x(1) ) / 2.0 + -5.0 * x(2);
	y(1) = -2.0 * x(0) - 3.0 * x(1) + -4.0 * x(2);

	return y;
}

bool checkFunction(
		const Eigen::Matrix<double, DOMAIN_DIM_, 1>& x,
		const Eigen::Matrix<double, RANGE_DIM_, 1>& y) {

	Eigen::Matrix<double, RANGE_DIM_, 1> yExact = func<double>(x);

	return y.isApprox(yExact);
}

bool checkJacobian(
		const Eigen::Matrix<double, DOMAIN_DIM_, 1>& x,
		const Eigen::Matrix<double, RANGE_DIM_, DOMAIN_DIM_>& sparsityMat,
		const Eigen::Matrix<double, RANGE_DIM_, DOMAIN_DIM_>& J) {

	Eigen::Matrix<double, RANGE_DIM_, DOMAIN_DIM_> JExact;
	JExact <<
			0.5, 	x(1), 	-5,
			-2.0,	-3.0,	-4;

	return J.array().isApprox(JExact.array()*sparsityMat.array());
}

bool checkHessian(const Eigen::Matrix<double, DOMAIN_DIM_, 1>& x,
		const Eigen::Matrix<double, RANGE_DIM_, DOMAIN_DIM_>& sparsityMat,
		const Eigen::Matrix<double, DOMAIN_DIM_, DOMAIN_DIM_>& H,
		const size_t outputIndex) {

	Eigen::Matrix<double, DOMAIN_DIM_, DOMAIN_DIM_> HExact;
	Eigen::Matrix<double, DOMAIN_DIM_, DOMAIN_DIM_> sparsity;

	switch (outputIndex) {
	case 0:
		HExact <<
			0.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 0.0;
		break;

	case 1:
		HExact <<
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0;
		break;
	}

	sparsity = sparsityMat.row(outputIndex).transpose() * sparsityMat.row(outputIndex);

	return H.array().isApprox(HExact.array()*sparsity.array());
}


TEST(testCppADCG_Simple_linux, cppadcg_linux)
{

    typedef ocs2::CppAdCodeGenInterface<DOMAIN_DIM_, RANGE_DIM_> cppadcg_interface_t;

    typedef cppadcg_interface_t::ad_dynamic_vector_t	ad_dynamic_vector_t;
    typedef cppadcg_interface_t::ad_funtion_t			ad_funtion_t;

    typedef cppadcg_interface_t::domain_vector_t		domain_vector_t;
    typedef cppadcg_interface_t::range_vector_t			range_vector_t;
    typedef cppadcg_interface_t::domain_matrix_t		domain_matrix_t;
    typedef cppadcg_interface_t::domain_range_matrix_t	domain_range_matrix_t;
    typedef cppadcg_interface_t::range_domain_matrix_t	range_domain_matrix_t;

    /***************************************************************************
     *                               the model
     **************************************************************************/
    ad_funtion_t adfunc = [](
    		const ad_dynamic_vector_t& x,
    		ad_dynamic_vector_t& y) {
    	y = func<ad_dynamic_vector_t::Scalar>(x);
   	};

    /***************************************************************************
     *                              Sparsity Pattern
     **************************************************************************/
    range_domain_matrix_t sparsityPattern;
    sparsityPattern.setOnes();
    sparsityPattern.col(2).setZero();

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    cppadcg_interface_t cppAdCodeGenClass_(adfunc, sparsityPattern);
    cppAdCodeGenClass_.computeForwardModel(true);
    cppAdCodeGenClass_.computeJacobianModel(true);
    cppAdCodeGenClass_.computeHessianModel(true);
    cppAdCodeGenClass_.createModels("simple", libraryFolder);

    /***************************************************************************
     *                       load dynamic library
     **************************************************************************/
    // Note that createModels() both generates and loads the model. However, if
    // the model is already compiled the following method can be used to load it.
    cppAdCodeGenClass_.loadModels("simple", libraryFolder);

    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    bool success = true;
    domain_vector_t x;

    for (size_t it=0; it<1000; it++) {

    	x = domain_vector_t::Random() * 100.0;

    	range_vector_t funcValue;
    	success = cppAdCodeGenClass_.getFunctionValue(x, funcValue);
//    	// print out the result
//    	if (success) {
//    		std::cout << "Function: " << std::endl;
//    		std::cout << funcValue << std::endl;
//    	}

    	success &= checkFunction(x, funcValue);
    	ASSERT_TRUE(success);

    	domain_range_matrix_t jacobian;
    	success = cppAdCodeGenClass_.getJacobian(x, jacobian);
//    	// print out the result
//    	if (success) {
//    		std::cout << "jacobian: " << std::endl;
//    		std::cout << jacobian.transpose() << std::endl;
//    	}

    	success &= checkJacobian(x, sparsityPattern, jacobian.transpose());
    	ASSERT_TRUE(success);

    	for (size_t i=0; i<RANGE_DIM_; i++) {
    		domain_matrix_t hessian;
    		success = cppAdCodeGenClass_.getHessian(x, hessian, i);
//    		// print out the result
//    		if (success) {
//    			std::cout << "hessian[" << i << "]: " << std::endl;
//    			std::cout << hessian << std::endl;
//    		}

    		success &= checkHessian(x, sparsityPattern, hessian, i);
    		ASSERT_TRUE(success);
    	}

    }

}

} // end of namespace cppadcg_simple_test

#endif /* TESTCPPADCG_SIMPLE_LINUX_OCS2_H_ */
