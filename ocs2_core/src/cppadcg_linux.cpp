/*
 * cppadcg_linux.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: farbod
 */

#include <iostream>
#include <boost/filesystem.hpp>
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>


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


int main(int argc, char** argv)
{

    typedef ocs2::CppAdCodeGenInterface<DOMAIN_DIM_, RANGE_DIM_> cppadcg_interface_t;

    typedef cppadcg_interface_t::ad_dynamic_vector_t	ad_dynamic_vector_t;
    typedef cppadcg_interface_t::ad_funtion_t			ad_funtion_t;

    typedef cppadcg_interface_t::domain_vector_t		domain_vector_t;
    typedef cppadcg_interface_t::range_vector_t			range_vector_t;
    typedef cppadcg_interface_t::domain_matrix_t		domain_matrix_t;
    typedef cppadcg_interface_t::doamin_range_matrix_t	doamin_range_matrix_t;
    typedef cppadcg_interface_t::range_doamin_matrix_t	range_doamin_matrix_t;

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
    range_doamin_matrix_t sparsityPattern;
    sparsityPattern.setOnes();
    sparsityPattern.col(2).setZero();

    /***************************************************************************
     *               path to save the generated dynamic-library
     **************************************************************************/
    boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/cppadcg_linux";

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    cppadcg_interface_t cppAdCodeGenClass_(adfunc, sparsityPattern);
    cppAdCodeGenClass_.computeForwardModel(true);
    cppAdCodeGenClass_.computeJacobianModel(true);
    cppAdCodeGenClass_.computeHessianModel(true);
//    cppAdCodeGenClass_.createModels("nn", libraryFolder);
    cppAdCodeGenClass_.loadModels("nn", libraryFolder);

    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    bool success = true;
    domain_vector_t x;

    for (size_t it=0; it<1; it++) {

    	x = domain_vector_t::Random() * 100.0;

    	range_vector_t funcValue;
    	success = cppAdCodeGenClass_.getFunctionValue(x, funcValue);
    	// print out the result
    	if (success) {
    		std::cout << "Function: " << std::endl;
    		std::cout << funcValue << std::endl;
    	}

    	success &= checkFunction(x, funcValue);

    	doamin_range_matrix_t jacobian;
    	success = cppAdCodeGenClass_.getJacobian(x, jacobian);
    	// print out the result
    	if (success) {
    		std::cout << "jacobian: " << std::endl;
    		std::cout << jacobian.transpose() << std::endl;
    	}

    	success &= checkJacobian(x, sparsityPattern, jacobian.transpose());

    	for (size_t i=0; i<RANGE_DIM_; i++) {
    		domain_matrix_t hessian;
    		success = cppAdCodeGenClass_.getHessian(x, hessian, i);
    		// print out the result
    		if (success) {
    			std::cout << "hessian[" << i << "]: " << std::endl;
    			std::cout << hessian << std::endl;
    		}

    		success &= checkHessian(x, sparsityPattern, hessian, i);
    	}

    }

}





