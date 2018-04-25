/*
 * AutomaticDifferentiationBase.h
 *
 *  Created on: Apr 23, 2018
 *      Author: farbod
 */

#ifndef AUTOMATICDIFFERENTIATIONBASE_OCS2_H_
#define AUTOMATICDIFFERENTIATIONBASE_OCS2_H_

#include <functional>
#include <Eigen/Core>
#include <cppad/cg.hpp>

namespace ocs2{

template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
class AutomaticDifferentiationBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<SCALAR_T, DOMAIN_DIM, 1>			domain_vector_t;
	typedef Eigen::Matrix<SCALAR_T, DOMAIN_DIM, DOMAIN_DIM> domain_matrix_t;
	typedef Eigen::Matrix<SCALAR_T, RANGE_DIM, 1>			range_vector_t;
	typedef Eigen::Matrix<SCALAR_T, RANGE_DIM, RANGE_DIM> 	range_matrix_t;
	typedef Eigen::Matrix<SCALAR_T, RANGE_DIM, DOMAIN_DIM>	range_doamin_matrix_t;
	typedef Eigen::Matrix<SCALAR_T, DOMAIN_DIM, RANGE_DIM>	doamin_range_matrix_t;

	AutomaticDifferentiationBase() = default;

	virtual ~AutomaticDifferentiationBase() = default;

	inline size_t domain() {
		return DOMAIN_DIM;
	}

	inline size_t range() {
		return RANGE_DIM;
	}

	virtual void computeForwardModel(bool computeForwardModel) = 0;

	virtual void computeJacobianModel(bool computeJacobianModel) = 0;

	virtual void computeHessianModel(bool computeHessianModel) = 0;

	virtual void createModels(
			const std::string& modelName,
			const std::string& libraryFolder = "",
			const bool verbose = true) = 0;

	virtual void loadModels(
			const std::string& modelName,
			const std::string& libraryFolder = "",
			const bool verbose = true) = 0;

	virtual void getSparsityPattern(
			range_doamin_matrix_t& sparsityPattern) const = 0;

	virtual bool getFunctionValue(
			const domain_vector_t& x,
			range_vector_t& funcValue) = 0;

	virtual bool getJacobian(
			const domain_vector_t& x,
			doamin_range_matrix_t& jacobian) = 0;

	virtual bool getHessian(
			const domain_vector_t& x,
			domain_matrix_t& hessian, size_t outputIndex=0) = 0;


protected:
	inline void extractSparsityPattern(
			const range_doamin_matrix_t& sparsityPattern,
			std::vector<size_t>& rowsJacobian,
			std::vector<size_t>& colsJacobian,
			std::vector<size_t>& rowsHessian,
			std::vector<size_t>& colsHessian)  {

		// Jacobain
		rowsJacobian.clear();
		colsJacobian.clear();

		for (size_t i=0; i<RANGE_DIM; i++)
			for (size_t j=0; j<DOMAIN_DIM; j++)
				if (sparsityPattern(i,j)>0) {
					rowsJacobian.push_back(i);
					colsJacobian.push_back(j);
				}

		// Hessian
		domain_vector_t sparsityPatternSum = sparsityPattern.colwise().maxCoeff().transpose();

		rowsHessian.clear();
		colsHessian.clear();

		for (size_t i=0; i<DOMAIN_DIM; i++)
			for (size_t j=i; j<DOMAIN_DIM; j++)
				if (sparsityPatternSum(i)>0 && sparsityPatternSum(j)>0) {
					rowsHessian.push_back(i);
					colsHessian.push_back(j);
				}
	}

};

} // namespace ocs2

#endif /* AUTOMATICDIFFERENTIATIONBASE_OCS2_H_ */
