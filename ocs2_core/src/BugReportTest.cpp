/*
 * BugReportTest.cpp
 *
 *  Created on: Apr 29, 2018
 *      Author: farbod
 */

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <cppad/cg.hpp>

int main(void) {

	typedef double scalar_t;
	typedef CppAD::cg::CG<scalar_t> 	ad_base_t;
    typedef CppAD::AD<ad_base_t> 		ad_t;
    typedef CppAD::ADFun<ad_base_t> 	ad_fun_t;

    const size_t DOMAIN_DIM  = 3;
    const size_t RANGE_DIM   = 1;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // set and declare independent variables and start tape recording
    Eigen::Matrix<ad_t, Eigen::Dynamic, 1> x(DOMAIN_DIM);
    x.setZero();
    CppAD::Independent(x);

    // dependent variable vector
    Eigen::Matrix<ad_t, Eigen::Dynamic, 1> y(RANGE_DIM);

    // the model equation
    y(0) = ( x(0) / 1.0 + x(1) * x(1) ) / 2.0 + -5.0 * x(2);
//    y(1) = -2.0 * x(0) - 3.0 * x(1) + -4.0 * x(2);

    // create f: x -> y and stop tape recording
    ad_fun_t fun(x, y);

    // Optimize the operation sequence
    fun.optimize();

    /***************************************************************************
     *                              Sparsity Pattern
     **************************************************************************/
    std::vector<size_t> rows;
    std::vector<size_t> cols;

    rows.push_back(0);
    cols.push_back(0);

    rows.push_back(0);
    cols.push_back(1);

    rows.push_back(1);
    cols.push_back(1);

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    // generates source code
    CppAD::cg::ModelCSourceGen<scalar_t> cgen(fun, "model");
	cgen.setCreateSparseHessian(true);
	cgen.setCustomSparseHessianElements(rows, cols);

    CppAD::cg::ModelLibraryCSourceGen<scalar_t> libcgen(cgen);

    // compile source code
    const std::string libraryName = "cppad_bug_model";
    CppAD::cg::DynamicModelLibraryProcessor<scalar_t> p(libcgen, libraryName);

    CppAD::cg::GccCompiler<scalar_t> compiler;
    std::unique_ptr<CppAD::cg::DynamicLib<scalar_t>> dynamicLib(p.createDynamicLibrary(compiler));


    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/

    std::unique_ptr<CppAD::cg::GenericModel<scalar_t>> model (dynamicLib->model("model"));

    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic> xv(DOMAIN_DIM);
    xv << 2.5, 3.5, 0.0;

    if (model->isSparseHessianAvailable()) {

    	Eigen::Matrix<scalar_t, 1, Eigen::Dynamic> weight(RANGE_DIM);
    	Eigen::Matrix<scalar_t, 1, Eigen::Dynamic> hess(DOMAIN_DIM*DOMAIN_DIM);
    	Eigen::Matrix<scalar_t, DOMAIN_DIM, DOMAIN_DIM> hessMat;

    	for (size_t i=0; i<RANGE_DIM; i++ ) {

    		weight.setZero();
    		weight(i) = 1.0;

    		model->SparseHessian(xv, weight, hess);

    		hessMat = Eigen::Map<Eigen::Matrix<scalar_t, DOMAIN_DIM, DOMAIN_DIM>> (hess.data());

    		// print out the result
    		std::cout << "hessMat[" << i << "]: " << std::endl;
    		std::cout << hessMat << std::endl;
    	}
    }
}



